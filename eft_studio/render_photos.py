import os
import struct
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image

# Repository root (this file is in <repo>/eft_studio/), so paths work wherever the repo is cloned
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def load_stl(stl_path):
    with open(stl_path, 'rb') as f:
        f.seek(80)
        count = struct.unpack('<I', f.read(4))[0]
        data = f.read()

    triangles = np.frombuffer(data, dtype=np.dtype([
        ('normal', np.float32, 3),
        ('v0', np.float32, 3),
        ('v1', np.float32, 3),
        ('v2', np.float32, 3),
        ('attr', np.uint16, 1)
    ]))

    v0 = triangles['v0']
    v1 = triangles['v1']
    v2 = triangles['v2']
    normals = triangles['normal']
    
    # Scale to meters and center
    all_v = np.vstack([v0, v1, v2])
    center = (all_v.min(axis=0) + all_v.max(axis=0)) / 2.0
    
    v0_m = (v0 - center) * 0.001
    v1_m = (v1 - center) * 0.001
    v2_m = (v2 - center) * 0.001
    
    # Offset so landing gear sits at Y=0 (bottom was -0.325m)
    v0_m[:, 1] += 0.325
    v1_m[:, 1] += 0.325
    v2_m[:, 1] += 0.325
    
    # In Matplotlib 3D: let's map X->X, Y->Z (up), Z->Y
    verts = np.stack([
        np.stack([v0_m[:, 0], v0_m[:, 2], v0_m[:, 1]], axis=1),
        np.stack([v1_m[:, 0], v1_m[:, 2], v1_m[:, 1]], axis=1),
        np.stack([v2_m[:, 0], v2_m[:, 2], v2_m[:, 1]], axis=1)
    ], axis=1)
    
    return verts, normals

def render_studio_view(verts, normals, elev, azim, filename, title="EFT E610P Digital Twin", bg_color="#0d111a", text_color="#00e5ff"):
    fig = plt.figure(figsize=(16, 10), facecolor=bg_color)
    ax = fig.add_subplot(111, projection='3d', facecolor=bg_color)
    
    # Subsample for smooth high quality polygon rendering
    step = 4
    sub_verts = verts[::step]
    sub_normals = normals[::step]
    
    # Compute lighting (Directional Key Light from top-right-front)
    light_dir = np.array([0.5, 0.7, 0.5])
    light_dir = light_dir / np.linalg.norm(light_dir)
    
    # Calculate diffuse intensity
    norm_3d = np.stack([sub_normals[:, 0], sub_normals[:, 2], sub_normals[:, 1]], axis=1)
    norm_len = np.linalg.norm(norm_3d, axis=1, keepdims=True)
    norm_len[norm_len == 0] = 1.0
    norm_unit = norm_3d / norm_len
    
    intensity = np.clip(np.abs(np.dot(norm_unit, light_dir)), 0.15, 1.0)
    
    # Generate colors (Carbon fiber dark slate with specular highlight)
    colors = np.zeros((len(sub_verts), 4))
    colors[:, 0] = 0.12 + 0.35 * intensity  # R
    colors[:, 1] = 0.14 + 0.38 * intensity  # G
    colors[:, 2] = 0.18 + 0.42 * intensity  # B
    colors[:, 3] = 0.95                     # Alpha
    
    # Create 3D PolyCollection
    poly = Poly3DCollection(sub_verts, facecolors=colors, edgecolors='none', linewidths=0)
    ax.add_collection3d(poly)
    
    # Add Rotors / Propeller Discs (6 arms at R=0.5854m, Z=0.6128m)
    r_arm = 0.5854
    h_arm = 0.6128
    prop_rad = 0.28
    for deg in [0, 60, 120, 180, 240, 300]:
        rad = math.radians(deg)
        cx = r_arm * math.cos(rad)
        cy = r_arm * math.sin(rad)
        theta = np.linspace(0, 2 * np.pi, 30)
        px = cx + prop_rad * np.cos(theta)
        py = cy + prop_rad * np.sin(theta)
        pz = np.full_like(px, h_arm + 0.02)
        ax.plot(px, py, pz, color='#00e5ff', alpha=0.6, linewidth=1.5)
        # Motor hub
        ax.plot([cx, cx], [cy, cy], [h_arm - 0.02, h_arm + 0.02], color='#ffaa00', linewidth=4)
        
    # Add 10L Spray Payload Tank outline (White / Translucent box)
    tx = [-0.19, 0.19, 0.19, -0.19, -0.19]
    ty = [-0.19, -0.19, 0.19, 0.19, -0.19]
    ax.plot(tx, ty, [0.22]*5, color='#ffffff', alpha=0.8, linewidth=1.8)
    ax.plot(tx, ty, [0.38]*5, color='#ffffff', alpha=0.8, linewidth=1.8)
    
    # Add TDCR Continuum Manipulator Arm (Bending spline under tank)
    t = np.linspace(0, 1, 20)
    bend_rad = math.radians(25)
    arm_x = np.sin(bend_rad * t**2) * 0.25 * t
    arm_y = np.sin(bend_rad * t**2) * 0.1 * t
    arm_z = 0.12 - np.cos(bend_rad * t**2) * 0.28 * t
    ax.plot(arm_x, arm_y, arm_z, color='#00ffaa', linewidth=3.5, label='TDCR Continuum Arm')
    
    # End effector nozzle
    ax.scatter([arm_x[-1]], [arm_y[-1]], [arm_z[-1]], color='#ff6600', s=60)
    
    # Camera Viewpoint
    ax.view_init(elev=elev, azim=azim)
    
    # Scale and Limits
    ax.set_xlim([-0.85, 0.85])
    ax.set_ylim([-0.85, 0.85])
    ax.set_zlim([0.0, 0.85])
    
    # Studio Styling - Hide ugly axes box
    ax.xaxis.set_pane_color((0, 0, 0, 0))
    ax.yaxis.set_pane_color((0, 0, 0, 0))
    ax.zaxis.set_pane_color((0, 0, 0, 0))
    ax.grid(False)
    ax.set_axis_off()
    
    # Add Studio Ground Grid
    grid_pts = np.linspace(-0.8, 0.8, 9)
    for g in grid_pts:
        ax.plot([g, g], [-0.8, 0.8], [0, 0], color='#1a2436', linewidth=0.8, alpha=0.6)
        ax.plot([-0.8, 0.8], [g, g], [0, 0], color='#1a2436', linewidth=0.8, alpha=0.6)
        
    # Title & Telemetry Header
    plt.suptitle(title, fontsize=18, fontweight='bold', color='#ffffff', y=0.94, fontfamily='sans-serif')
    ax.text2D(0.05, 0.05, 'EFT E610P AGRI-HEXACOPTER • PX4 SITL DIGITAL TWIN\nWheelbase: 1170mm | Payload: 10L Tank + TDCR Arm | Rotors: 6x 24" CW/CCW', 
              transform=ax.transAxes, color=text_color, fontsize=10, family='monospace')
    
    plt.tight_layout()
    out_path = os.path.join(REPO_ROOT, 'renders', filename)
    plt.savefig(out_path, dpi=200, facecolor=bg_color, edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f'Rendered photo: {out_path}')

def main():
    stl_path = os.path.join(REPO_ROOT, 'px4_customizations', 'models', 'eft_e610p', 'meshes', 'frame.STL')
    print('Loading 560k polygon STL CAD model...')
    verts, normals = load_stl(stl_path)
    
    # 1. Hero 3/4 Perspective Studio Shot
    render_studio_view(verts, normals, elev=22, azim=-55, 
                       filename='eft_e610p_hero_34_shot.png',
                       title='EFT E610P Digital Twin — Hero 3/4 Perspective')
                       
    # 2. Front Elevation Studio Shot
    render_studio_view(verts, normals, elev=12, azim=-90, 
                       filename='eft_e610p_front_elevation.png',
                       title='EFT E610P Digital Twin — Front Elevation')

    # 3. Side Profile Studio Shot
    render_studio_view(verts, normals, elev=10, azim=0, 
                       filename='eft_e610p_side_profile.png',
                       title='EFT E610P Digital Twin — Side Profile')

    # 4. Top-Down Plan View Shot
    render_studio_view(verts, normals, elev=82, azim=-90, 
                       filename='eft_e610p_top_plan_view.png',
                       title='EFT E610P Digital Twin — Top Plan View (Hexacopter Rotor Array)')

    # 5. Underside Payload & Continuum Arm Shot
    render_studio_view(verts, normals, elev=-25, azim=-45, 
                       filename='eft_e610p_underside_payload.png',
                       title='EFT E610P Digital Twin — Underside 10L Tank & TDCR Arm')

    # 6. Clean White Studio High-Key Shot
    render_studio_view(verts, normals, elev=25, azim=-60, 
                       filename='eft_e610p_clean_white_studio.png',
                       title='EFT E610P Digital Twin — Clean White Studio',
                       bg_color='#f5f7fb', text_color='#0070f3')

if __name__ == '__main__':
    main()
