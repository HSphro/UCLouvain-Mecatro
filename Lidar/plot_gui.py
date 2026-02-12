#!/usr/bin/env python3
"""
Tkinter GUI for realtime LIDAR plotting.
Reads lines like: "Angle: 123.45 deg   Distance: 678.90 mm" from stdin (or load from file)

Controls:
 - Start / Stop reading
 - Save snapshot
 - Set axis range and refresh interval
 - Clear current scan

Run examples:
  ./lidar_ankit | python3 Lidar/plot_gui.py
  python3 Lidar/plot_gui.py --file capture.txt

"""
import sys
import re
import threading
import time
import argparse
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.pyplot as plt
import math
from Position_angle_Lidar import assign_clusters_to_beacons, trilaterate, compute_robot_orientation
from Position_angle_Lidar import B1_REF, B2_REF, B3_REF

PAT = re.compile(r"Angle:\s*([0-9]+(?:\.[0-9]*)?)\s*deg\s+Distance:\s*([0-9]+(?:\.[0-9]*)?)\s*mm(?:\s+Quality:\s*([0-9]+))?")

class LidarGUI:
    def __init__(self, master, source_file=None):
        self.master = master
        self.source_file = source_file
        self.master.title('LIDAR Realtime GUI')

        self.distances = np.zeros(360, dtype=float)
        self.seen = np.zeros(360, dtype=bool)
        self.reflectivity = np.zeros(360, dtype=float)  # Store signal strength (0-63)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.reader_thread = None
        self.reading = False

        # Default params
        self.interval_ms = tk.IntVar(value=80)
        self.axis_range = tk.StringVar(value='1500')
        # Distance filtering (mm) for clustering
        self.min_dist = tk.DoubleVar(value=50.0)
        self.max_dist = tk.DoubleVar(value=4500.0)
        # Reflectivity filtering (quality 0-63 from beacon signal strength)
        self.min_quality = tk.DoubleVar(value=0)  # Minimum reflectivity threshold
        # Clustering / cylinder params
        self.cylinder_radius = tk.DoubleVar(value=40.0)  # expected cylinder radius (mm)
        self.cluster_eps = tk.DoubleVar(value=60.0)      # DBSCAN eps (mm). If 0 use derived from radius
        self.cluster_tol = tk.DoubleVar(value=30.0)      # tolerance percent for fitted radius
        self.cluster_minpts = tk.IntVar(value=5)

        # Positioning state
        self.detected_clusters = []  # Store last detections
        self.robot_position = None
        self.robot_orientation = None
        self.prev_B1 = B1_REF.copy()
        self.prev_B2 = B2_REF.copy()
        self.prev_B3 = B3_REF.copy()

        # Rotation detection state
        self.last_angle = None
        self.rotation_ready = False

        self._build_ui()
        self._build_plot()
        self._schedule_draw()

    def _build_ui(self):
        ctrl = ttk.Frame(self.master)
        ctrl.pack(side='top', fill='x', padx=6, pady=6)

        self.start_btn = ttk.Button(ctrl, text='Start', command=self.start)
        self.start_btn.grid(row=0, column=0)
        self.stop_btn = ttk.Button(ctrl, text='Stop', command=self.stop)
        self.stop_btn.grid(row=0, column=1)
        self.clear_btn = ttk.Button(ctrl, text='Clear', command=self.clear)
        self.clear_btn.grid(row=0, column=2)

        ttk.Label(ctrl, text='Range (mm)').grid(row=0, column=3, padx=(10,0))
        self.range_entry = ttk.Entry(ctrl, width=8, textvariable=self.axis_range)
        self.range_entry.grid(row=0, column=4)

        ttk.Label(ctrl, text='Cyl R (mm)').grid(row=0, column=9, padx=(10,0))
        self.cyl_entry = ttk.Entry(ctrl, width=6, textvariable=self.cylinder_radius)
        self.cyl_entry.grid(row=0, column=10)

        ttk.Label(ctrl, text='Tol (%)').grid(row=0, column=11, padx=(6,0))
        self.tol_entry = ttk.Entry(ctrl, width=6, textvariable=self.cluster_tol)
        self.tol_entry.grid(row=0, column=12)

        ttk.Label(ctrl, text='Cluster eps').grid(row=0, column=13, padx=(6,0))
        self.eps_entry = ttk.Entry(ctrl, width=6, textvariable=self.cluster_eps)
        self.eps_entry.grid(row=0, column=14)

        ttk.Label(ctrl, text='Interval (ms)').grid(row=0, column=5, padx=(10,0))
        self.int_entry = ttk.Entry(ctrl, width=6, textvariable=self.interval_ms)
        self.int_entry.grid(row=0, column=6)

        self.save_btn = ttk.Button(ctrl, text='Save Snapshot', command=self.save_snapshot)
        self.save_btn.grid(row=0, column=7, padx=(10,0))

        ttk.Label(ctrl, text='Min Dist (mm)').grid(row=0, column=15, padx=(6,0))
        self.min_entry = ttk.Entry(ctrl, width=6, textvariable=self.min_dist)
        self.min_entry.grid(row=0, column=16)

        ttk.Label(ctrl, text='Max Dist (mm)').grid(row=0, column=17, padx=(6,0))
        self.max_entry = ttk.Entry(ctrl, width=6, textvariable=self.max_dist)
        self.max_entry.grid(row=0, column=18)

        ttk.Label(ctrl, text='Min Quality').grid(row=0, column=19, padx=(6,0))
        self.quality_entry = ttk.Entry(ctrl, width=6, textvariable=self.min_quality)
        self.quality_entry.grid(row=0, column=20)

        if not self.source_file:
            ttk.Label(ctrl, text='(reading stdin)').grid(row=0, column=8, padx=(10,0))
        else:
            ttk.Label(ctrl, text=f'File: {self.source_file}').grid(row=0, column=8, padx=(10,0))

    def _build_plot(self):
        self.fig, (self.ax, self.ax_global) = plt.subplots(1, 2, figsize=(14, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().pack(side='top', fill='both', expand=1)
        toolbar = NavigationToolbar2Tk(self.canvas, self.master)
        toolbar.update()

    def start(self):
        if self.reading:
            return
        self.stop_event.clear()
        if self.source_file:
            self.reader_thread = threading.Thread(target=self._read_file, daemon=True)
        else:
            self.reader_thread = threading.Thread(target=self._read_stdin, daemon=True)
        self.reader_thread.start()
        self.reading = True

    def stop(self):
        if not self.reading:
            return
        self.stop_event.set()
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)
        self.reading = False

    def clear(self):
        with self.lock:
            self.distances[:] = 0
            self.seen[:] = False
            self.reflectivity[:] = 0

    def save_snapshot(self):
        path = filedialog.asksaveasfilename(defaultextension='.png', filetypes=[('PNG','*.png'), ('All','*.*')])
        if not path:
            return
        try:
            self.fig.savefig(path, dpi=150, bbox_inches='tight')
            messagebox.showinfo('Saved', f'Snapshot saved to {path}')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to save: {e}')

    def _read_stdin(self):
        # Read line-by-line from stdin
        while not self.stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.01)
                continue
            m = PAT.search(line)
            if not m:
                continue
            try:
                ang = float(m.group(1))
                dist = float(m.group(2))
                quality = float(m.group(3)) if m.group(3) else 0.0
            except Exception:
                continue
            if dist <= 0:
                continue
            idx = int(ang + 0.5) % 360
            with self.lock:
                prev = self.last_angle
                self.distances[idx] = dist
                self.reflectivity[idx] = quality
                self.seen[idx] = True
                # detect wrap-around (rotation completed)
                if prev is not None:
                    # if angle decreases significantly we likely wrapped
                    if idx + 300 < prev:  # allow jitter; wrapped (e.g., prev ~350, idx ~2)
                        self.rotation_ready = True
                self.last_angle = idx

    def _read_file(self):
        try:
            with open(self.source_file, 'r', errors='ignore') as f:
                for line in f:
                    if self.stop_event.is_set():
                        break
                    m = PAT.search(line)
                    if not m:
                        continue
                    try:
                        ang = float(m.group(1))
                        dist = float(m.group(2))
                        quality = float(m.group(3)) if m.group(3) else 0.0
                    except Exception:
                        continue
                    if dist <= 0:
                        continue
                    idx = int(ang + 0.5) % 360
                    with self.lock:
                        prev = self.last_angle
                        self.distances[idx] = dist
                        self.reflectivity[idx] = quality
                        self.seen[idx] = True
                        if prev is not None:
                            if idx + 300 < prev:
                                self.rotation_ready = True
                        self.last_angle = idx
                    time.sleep(0.005)
        except Exception as e:
            messagebox.showerror('Error', f'Failed reading file: {e}')

    def _schedule_draw(self):
        ms = max(10, int(self.interval_ms.get()))
        self._draw()
        self.master.after(ms, self._schedule_draw)

    def _draw(self):
        with self.lock:
            d = self.distances.copy()
            s = self.seen.copy()
            q = self.reflectivity.copy()
        
        # ========== LEFT PLOT: LIDAR SCAN (POLAR) ==========
        self.ax.cla()
        if s.any():
            angles = np.deg2rad(np.arange(360))
            x = d * np.cos(angles)
            y = d * np.sin(angles)
            
            # Apply quality/reflectivity filter
            try:
                min_q = float(self.min_quality.get())
            except Exception:
                min_q = 0.0
            
            # Filter by quality first
            quality_mask = q >= min_q
            display_mask = s & quality_mask
            
            # All seen points for display
            if display_mask.any():
                X_all = np.column_stack((x[display_mask], y[display_mask]))
                sc = self.ax.scatter(X_all[:,0], X_all[:,1], c=d[display_mask], cmap='viridis', s=18)
            else:
                sc = None
            
            cbar = self.fig.axes[-1] if len(self.fig.axes) > 1 else None
            # Add colorbar safely
            try:
                if hasattr(self, '_cbar'):
                    self._cbar.remove()
                if sc is not None:
                    self._cbar = self.fig.colorbar(sc, ax=self.ax, shrink=0.8)
                    self._cbar.set_label('Distance (mm)')
            except Exception:
                pass
            # --- Clustering and cylinder detection ---
            self.detected_clusters = []
            try:
                # distance bounds for clustering
                try:
                    min_d = float(self.min_dist.get())
                    max_d = float(self.max_dist.get())
                except Exception:
                    min_d = 0.0
                    max_d = 1e9

                # Apply both distance and quality filters for clustering
                mask = display_mask & (d >= min_d) & (d <= max_d)

                # Only run clustering when a rotation has been detected
                if self.rotation_ready:
                    pts = np.column_stack((x[mask], y[mask])) if mask.any() else np.zeros((0,2))

                    # choose eps: use provided cluster_eps if > 0 else derive from radius
                    eps_val = float(self.cluster_eps.get())
                    if eps_val <= 0:
                        eps_val = max(8.0, float(self.cylinder_radius.get()) * 0.4)
                    minpts = int(max(3, self.cluster_minpts.get()))

                    labels = self._cluster_points(pts, eps_val, minpts) if pts.shape[0] > 0 else np.array([], dtype=int)
                    unique_labels = sorted(set(labels))
                    for lbl in unique_labels:
                        if lbl == -1:
                            continue
                        idxs = np.where(labels == lbl)[0]
                        cpts = pts[idxs]
                        if cpts.shape[0] < 3:
                            continue
                        try:
                            center, R_est = self._fit_circle(cpts)
                        except Exception:
                            continue
                        if not (np.isfinite(R_est) and R_est > 1.0):
                            continue
                        expR = float(self.cylinder_radius.get())
                        tol = float(self.cluster_tol.get()) / 100.0
                        if expR > 0 and abs(R_est - expR) / expR > tol:
                            continue

                        self.detected_clusters.append(center)

                        # compute visible arc for display
                        angs = np.arctan2(cpts[:,1] - center[1], cpts[:,0] - center[0])
                        angs = np.mod(angs, 2*math.pi)
                        angs_sorted = np.sort(angs)
                        gaps = np.diff(np.concatenate((angs_sorted, angs_sorted[:1] + 2*math.pi)))
                        if len(angs_sorted) > 0:
                            max_gap_idx = np.argmax(gaps)
                            start = angs_sorted[(max_gap_idx + 1) % len(angs_sorted)]
                            end = angs_sorted[max_gap_idx]
                            if end < start:
                                end += 2*math.pi
                            thetas = np.linspace(start, end, 120)
                            arc_x = center[0] + R_est * np.cos(thetas)
                            arc_y = center[1] + R_est * np.sin(thetas)
                            self.ax.plot(arc_x, arc_y, color='red', linewidth=2)
                            self.ax.plot([center[0]], [center[1]], marker='x', color='red')

                    # finished processing this rotation — clear raw scan to remove residues
                    with self.lock:
                        self.distances[:] = 0
                        self.seen[:] = False
                        self.reflectivity[:] = 0
                        self.rotation_ready = False
                        self.last_angle = None
                else:
                    # Not yet a completed rotation; keep accumulating
                    pass
            except Exception:
                pass
        else:
            self.ax.text(0.5, 0.5, 'Waiting for data...', ha='center', va='center', transform=self.ax.transAxes)

        # axis limits
        try:
            rng = float(self.axis_range.get())
        except Exception:
            rng = None
        if not rng or rng <= 0:
            if display_mask.any() if 'display_mask' in locals() else s.any():
                if 'display_mask' in locals() and display_mask.any():
                    rng = max(1000.0, np.nanmax(d[display_mask]) * 1.2)
                elif s.any():
                    rng = max(1000.0, np.nanmax(d[s]) * 1.2)
                else:
                    rng = 1000.0
            else:
                rng = 1000.0
        self.ax.set_xlim(-rng, rng)
        self.ax.set_ylim(-rng, rng)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        pts = int(np.sum(display_mask)) if 'display_mask' in locals() else int(np.sum(s))
        self.ax.set_title(f'LIDAR Scan | Points: {pts}')
        
        # ========== RIGHT PLOT: GLOBAL POSITIONING ==========
        self.ax_global.cla()
        
        # Draw beacon reference points (fixed)
        self.ax_global.plot([B1_REF[0]], [B1_REF[1]], marker='o', color='blue', markersize=10, label='B1 (ref)')
        self.ax_global.plot([B2_REF[0]], [B2_REF[1]], marker='s', color='green', markersize=10, label='B2 (ref)')
        self.ax_global.plot([B3_REF[0]], [B3_REF[1]], marker='^', color='purple', markersize=10, label='B3 (ref)')
        
        # Draw detected clusters in global frame
        for i, cluster_center in enumerate(self.detected_clusters):
            self.ax_global.plot([cluster_center[0]], [cluster_center[1]], marker='*', color='orange', markersize=15)
            self.ax_global.text(cluster_center[0]+50, cluster_center[1]+50, f'C{i+1}', fontsize=9)
        
        # Compute and display robot position if 3 clusters detected
        if len(self.detected_clusters) >= 3:
            try:
                # Assign clusters to beacons
                B1, B2, B3 = assign_clusters_to_beacons(
                    self.detected_clusters[:3], self.prev_B1, self.prev_B2, self.prev_B3
                )
                
                # Update previous positions for temporal continuity
                self.prev_B1 = B1.copy()
                self.prev_B2 = B2.copy()
                self.prev_B3 = B3.copy()
                
                # Compute distances from robot to beacons
                d1 = np.linalg.norm(B1)
                d2 = np.linalg.norm(B2)
                d3 = np.linalg.norm(B3)
                
                # Trilaterate robot position
                self.robot_position = trilaterate(B1_REF, B2_REF, B3_REF, d1, d2, d3)
                
                # Compute robot orientation
                self.robot_orientation = compute_robot_orientation(B1, B2)
                
                # Draw robot position
                self.ax_global.plot([self.robot_position[0]], [self.robot_position[1]], 
                                  marker='D', color='red', markersize=12, label='Robot')
                
                # Draw robot orientation arrow
                arrow_len = 200
                arrow_x = self.robot_position[0] + arrow_len * np.cos(self.robot_orientation)
                arrow_y = self.robot_position[1] + arrow_len * np.sin(self.robot_orientation)
                self.ax_global.arrow(self.robot_position[0], self.robot_position[1],
                                   arrow_x - self.robot_position[0], arrow_y - self.robot_position[1],
                                   head_width=100, head_length=80, fc='red', ec='red', linewidth=2)
                
                # Add position/orientation text
                theta_deg = np.rad2deg(self.robot_orientation) % 360
                info_text = f"Robot: ({self.robot_position[0]:.1f}, {self.robot_position[1]:.1f}) mm\nθ: {theta_deg:.1f}°"
                self.ax_global.text(0.02, 0.98, info_text, transform=self.ax_global.transAxes,
                                  fontsize=10, verticalalignment='top',
                                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            except Exception as e:
                self.ax_global.text(0.5, 0.5, f'Positioning error: {str(e)[:30]}', 
                                  ha='center', va='center', transform=self.ax_global.transAxes)
        else:
            status_text = f"Waiting for 3 clusters... ({len(self.detected_clusters)}/3)"
            self.ax_global.text(0.5, 0.5, status_text, ha='center', va='center', 
                              transform=self.ax_global.transAxes, fontsize=12)
        
        # Set global plot limits
        global_range = 4500
        self.ax_global.set_xlim(-500, 4500)
        self.ax_global.set_ylim(-500, 2500)
        self.ax_global.set_aspect('equal')
        self.ax_global.set_xlabel('X (mm)')
        self.ax_global.set_ylabel('Y (mm)')
        self.ax_global.set_title('Robot Positioning (Global Frame)')
        self.ax_global.legend(loc='upper right', fontsize=8)
        self.ax_global.grid(True, alpha=0.3)
        
        self.canvas.draw_idle()

    def _cluster_points(self, pts, eps, min_samples=3):
        # Simple pure-numpy DBSCAN implementation
        if pts is None or len(pts) == 0:
            return np.array([], dtype=int)
        pts = np.asarray(pts)
        n = pts.shape[0]
        dists = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
        visited = np.zeros(n, dtype=bool)
        labels = -1 * np.ones(n, dtype=int)
        lbl = 0
        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True
            neighbors = list(np.where(dists[i] <= eps)[0])
            if len(neighbors) < min_samples:
                labels[i] = -1
                continue
            labels[i] = lbl
            seeds = [p for p in neighbors if p != i]
            while seeds:
                q = seeds.pop()
                if not visited[q]:
                    visited[q] = True
                    q_neighbors = list(np.where(dists[q] <= eps)[0])
                    if len(q_neighbors) >= min_samples:
                        for nn in q_neighbors:
                            if nn not in seeds:
                                seeds.append(nn)
                if labels[q] == -1:
                    labels[q] = lbl
            lbl += 1
        return labels

    def _fit_circle(self, pts):
        # Algebraic circle fit via linearization
        x = pts[:,0]
        y = pts[:,1]
        A = np.column_stack((-2*x, -2*y, np.ones_like(x)))
        b = -(x*x + y*y)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        a = sol[0]; bb = sol[1]; s = sol[2]
        center = np.array([a, bb])
        R = math.sqrt(max(0.0, a*a + bb*bb - s))
        return center, R

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', help='Read from capture file instead of stdin')
    args = parser.parse_args()

    root = tk.Tk()
    gui = LidarGUI(root, source_file=args.file)
    # Start reading immediately if stdin is piped or a file was provided
    piped = not sys.stdin.isatty()
    if piped or args.file:
        gui.start()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        gui.stop()

if __name__ == '__main__':
    main()
