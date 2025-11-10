"""
Video utilities for empirical evaluations
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
import io
from PIL import Image


class VideoRecorder:
    """Records video frames and error plots for wandb"""
    
    def __init__(self, width: int = 512, height: int = 512, 
                 camera_name: str = "frontview", 
                 max_history: int = None):
        self.width = width
        self.height = height
        self.camera_name = camera_name
        self.max_history = max_history  # None means keep all history
        
        # Store error history for plotting
        self.error_history: List[Tuple[float, float, float]] = []  # (x_error, y_error, z_error)
        self.time_history: List[int] = []
        self.frames: List[np.ndarray] = []
        
    def add_frame(self, env, x_error: float, y_error: float, z_error: float, step: int):
        """Add a frame to the video recording"""
        # Capture frame from environment using offscreen renderer
        # Note: requires has_offscreen_renderer=True
        try:
            # Try to get frame from camera observations first (more reliable)
            obs = env._get_observations()
            camera_key = f"{self.camera_name}_image"
            
            if camera_key in obs:
                frame = obs[camera_key].copy()
                # Camera obs are typically uint8 already
                if frame.dtype != np.uint8:
                    frame = (np.clip(frame, 0, 1) * 255).astype(np.uint8)
                # Flip vertically (robosuite camera obs are upside down by default)
                frame = np.flipud(frame)
                if step % 100 == 0:
                    print(f"Using camera obs: {camera_key}, shape={frame.shape}, dtype={frame.dtype}")
            else:
                if step % 100 == 0:
                    print(f"Camera key '{camera_key}' not found in obs. Keys available: {list(obs.keys())[:10]}")
                # Fallback: try sim.render
                frame = env.sim.render(
                    height=self.height,
                    width=self.width, 
                    camera_name=self.camera_name,
                    depth=False,
                    mode='offscreen'
                )
                # Convert to uint8 if needed (render returns float [0, 1])
                if frame.dtype != np.uint8:
                    frame = (np.clip(frame, 0, 1) * 255).astype(np.uint8)
            
            # Ensure 3 channels (RGB)
            if len(frame.shape) == 2:
                frame = np.stack([frame, frame, frame], axis=-1)
            elif frame.shape[2] == 4:  # RGBA
                frame = frame[:, :, :3]  # Remove alpha
            
            # Ensure dimensions are correct
            if frame.shape[0] == 0 or frame.shape[1] == 0:
                raise ValueError("Invalid frame dimensions")
            
            # Debug: print frame shape
            if step % 100 == 0:
                print(f"Captured frame at step {step}: shape={frame.shape}, dtype={frame.dtype}, range=[{frame.min()}, {frame.max()}]")
            
        except Exception as e:
            print(f"Warning: Could not capture frame: {e}")
            import traceback
            traceback.print_exc()
            # Create a dummy frame
            frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 128
        
        # Add to history (make sure to copy the frame!)
        self.error_history.append((x_error, y_error, z_error))
        self.time_history.append(step)
        self.frames.append(frame.copy())  # CRITICAL: Copy the frame, not just the reference!
        
        # Limit history size if max_history is set
        if self.max_history is not None and len(self.error_history) > self.max_history:
            self.error_history.pop(0)
            self.time_history.pop(0)
            self.frames.pop(0)
    
    def _create_error_plot(self) -> np.ndarray:
        """Create an error plot visualization"""
        if not self.error_history:
            # Return blank plot
            fig, ax = plt.subplots(figsize=(4, 3))
            ax.text(0.5, 0.5, 'No data yet', ha='center', va='center')
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
        else:
            # Create plot
            fig, ax = plt.subplots(figsize=(4, 3))
            
            # Extract error components
            x_errors = [e[0] for e in self.error_history]
            y_errors = [e[1] for e in self.error_history]
            z_errors = [e[2] for e in self.error_history]
            times = self.time_history
            
            # Plot each component
            ax.plot(times, x_errors, label='X', color='r', linewidth=1.5)
            ax.plot(times, y_errors, label='Y', color='g', linewidth=1.5)
            ax.plot(times, z_errors, label='Z', color='b', linewidth=1.5)
            
            # Add title with frame count
            ax.set_title(f'Position Error (frame {len(self.error_history)})')
            ax.set_xlabel('Global Step')
            ax.set_ylabel('Error (m)')
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            
            # Tight layout
            plt.tight_layout()
        
        # Convert figure to image array
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=100, bbox_inches='tight')
        buf.seek(0)
        img = Image.open(buf)
        img_array = np.array(img)
        plt.close(fig)
        
        return img_array
    
    def _create_error_plot_at_index(self, frame_idx: int, window_size: int = 150) -> np.ndarray:
        """Create error plot up to a specific frame index with rolling window"""
        if not self.error_history or frame_idx < 0:
            return None
        
        # Use rolling window of recent data
        start_idx = max(0, frame_idx + 1 - window_size)
        history_slice = self.error_history[start_idx:frame_idx + 1]
        times_slice = self.time_history[start_idx:frame_idx + 1]
        
        if not history_slice:
            return None
        
        # Create plot with high DPI and nice styling
        fig, ax = plt.subplots(figsize=(8, 6), facecolor='white')
        ax.set_facecolor('#f8f9fa')
        
        # Extract error components
        x_errors = [e[0] for e in history_slice]
        y_errors = [e[1] for e in history_slice]
        z_errors = [e[2] for e in history_slice]
        
        # Plot each component with nice colors and styling
        ax.plot(times_slice, x_errors, label='X Error', color='#e74c3c', linewidth=2.5, alpha=0.9)
        ax.plot(times_slice, y_errors, label='Y Error', color='#2ecc71', linewidth=2.5, alpha=0.9)
        ax.plot(times_slice, z_errors, label='Z Error', color='#3498db', linewidth=2.5, alpha=0.9)
        
        # Styling
        ax.set_title(f'Position Error Over Time (step {frame_idx + 1})', 
                    fontsize=14, fontweight='bold', pad=15)
        ax.set_xlabel('Global Step', fontsize=12, fontweight='bold')
        ax.set_ylabel('Error (m)', fontsize=12, fontweight='bold')
        
        # Legend styling
        ax.legend(loc='upper right', frameon=True, fancybox=True, shadow=True, 
                 fontsize=11, framealpha=0.95)
        
        # Grid styling
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        # Tick styling
        ax.tick_params(axis='both', which='major', labelsize=10)
        
        # Tight layout
        plt.tight_layout()
        
        # Convert figure to image array with higher DPI
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=200, bbox_inches='tight', facecolor='white', 
                   edgecolor='none', pad_inches=0.2)
        buf.seek(0)
        img = Image.open(buf)
        img_array = np.array(img)
        plt.close(fig)
        
        return img_array
    
    def create_combined_video(self) -> Optional[np.ndarray]:
        """Create combined video with robot view and error plot side by side"""
        if not self.frames:
            return None
        
        combined_frames = []
        
        # Get target dimensions from first frame
        target_height = self.frames[0].shape[0]
        target_width = self.frames[0].shape[1]
        
        # Pre-compute ALL plots at once using in-place line updates
        print(f"Pre-computing {len(self.frames)} plots...")
        all_plot_images = []
        
        # Create figure and axes with line objects that we'll update in-place
        fig = plt.figure(figsize=(8, 6), facecolor='white')
        ax = fig.add_subplot(111)
        ax.set_facecolor('#f8f9fa')
        
        # Set up axes limits and styling once
        max_time = max(self.time_history) if self.time_history else 1000
        ax.set_xlim(0, max_time)
        ax.set_ylim(-0.05, 0.05)  # Assume errors in this range
        ax.set_xlabel('Global Step', fontsize=12, fontweight='bold')
        ax.set_ylabel('Error (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        ax.tick_params(axis='both', which='major', labelsize=10)
        
        # Create empty line objects
        line_x, = ax.plot([], [], label='X Error', color='#e74c3c', linewidth=2.5, alpha=0.9)
        line_y, = ax.plot([], [], label='Y Error', color='#2ecc71', linewidth=2.5, alpha=0.9)
        line_z, = ax.plot([], [], label='Z Error', color='#3498db', linewidth=2.5, alpha=0.9)
        ax.legend(loc='upper right', frameon=True, fancybox=True, shadow=True, 
                 fontsize=11, framealpha=0.95)
        
        # Add title element
        title_text = ax.text(0.5, 0.95, '', transform=ax.transAxes, 
                            fontsize=14, fontweight='bold', ha='center', va='top')
        
        for i in range(len(self.frames)):
            # Update line data in-place (much faster than clearing and replotting)
            window_size = 150
            start_idx = max(0, i + 1 - window_size)
            history_slice = self.error_history[start_idx:i + 1]
            times_slice = self.time_history[start_idx:i + 1]
            
            if history_slice:
                x_errors = [e[0] for e in history_slice]
                y_errors = [e[1] for e in history_slice]
                z_errors = [e[2] for e in history_slice]
                
                # Update line data
                line_x.set_data(times_slice, x_errors)
                line_y.set_data(times_slice, y_errors)
                line_z.set_data(times_slice, z_errors)
                
                # Update axes limits to fit current data
                all_errors = x_errors + y_errors + z_errors
                if all_errors:
                    error_min, error_max = min(all_errors), max(all_errors)
                    error_range = error_max - error_min
                    padding = error_range * 0.1 if error_range > 0 else 0.01
                    ax.set_xlim(min(times_slice), max(times_slice))
                    ax.set_ylim(error_min - padding, error_max + padding)
                
                # Update title
                title_text.set_text(f'Position Error Over Time (step {i + 1})')
            
            # Render to buffer using canvas (much faster than savefig!)
            fig.canvas.draw()
            buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            error_plot = buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            
            # Resize to target dimensions
            plot_pil = Image.fromarray(error_plot)
            plot_pil = plot_pil.resize((target_width, target_height), Image.LANCZOS)
            error_plot_resized = np.array(plot_pil)
            
            # Ensure RGB
            if len(error_plot_resized.shape) == 2:  # Grayscale
                error_plot_resized = np.stack([error_plot_resized, error_plot_resized, error_plot_resized], axis=-1)
            elif error_plot_resized.shape[2] == 4:  # RGBA
                error_plot_resized = error_plot_resized[:, :, :3]
            
            all_plot_images.append(error_plot_resized)
            
            if (i + 1) % 50 == 0:
                print(f"  Computed {i + 1}/{len(self.frames)}")
        
        plt.close(fig)
        
        print("Combining frames...")
        # Now combine frames with pre-computed plots
        for i, robot_frame in enumerate(self.frames):
            # Ensure robot frame is RGB
            if robot_frame.shape[2] == 4:
                robot_frame = robot_frame[:, :, :3]
            elif len(robot_frame.shape) == 2:
                robot_frame = np.stack([robot_frame, robot_frame, robot_frame], axis=-1)
            
            # Combine with pre-computed plot
            combined = np.concatenate([robot_frame, all_plot_images[i]], axis=1)
            combined_frames.append(combined)
        
        # Convert to numpy array
        if combined_frames:
            return np.array(combined_frames)
        return None
    
    def reset(self):
        """Reset the recorder"""
        self.error_history.clear()
        self.time_history.clear()
        self.frames.clear()

