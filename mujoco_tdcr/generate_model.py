import math
import struct

# --- User-specified dimensions ---
R_disk = 0.0275       # 27.5 mm radius
t_disk = 0.006        # 6.0 mm disk thickness
R_tendon = 0.01766    # 17.66 mm from center
R_spring = 0.01725    # Center line of the spring folds
t_spring = 0.0016     # 1.6 mm thickness of strip
w_spring = 0.00538    # 5.38 mm depth/width of strip
d_spring = 0.01025    # 10.25 mm fold amplitude (half of 20.5 mm total width)
gap_spring = 0.0044   # 4.4 mm gap between adjacent curves/plates
H_gap = 0.0676        # 67.6 mm total height of the spring

# --- Derived and calculation parameters ---
num_plates = 12       # 12 horizontal plates fit exactly
spring_rgba = "0.9 0.9 0.9 1"  # White/light-gray 3D printed appearance

# Tendon positions at 45 degree offsets (NE, NW, SW, SE)
xy_tendon = R_tendon * math.cos(math.radians(45))  # ~0.0124885 m

# Z coordinates for Module 1
z_start_1 = 0.050 + 0.5 * t_disk # 0.053
z_plates_1 = [z_start_1 + 0.5 * t_spring + i * (t_spring + gap_spring) for i in range(num_plates)]
z_curves_1 = [0.5 * (z_plates_1[i] + z_plates_1[i+1]) for i in range(num_plates - 1)]

# Z coordinates for Module 2
z_start_2 = z_start_1 + H_gap + t_disk # 0.1266
z_plates_2 = [z_start_2 + 0.5 * t_spring + i * (t_spring + gap_spring) for i in range(num_plates)]
z_curves_2 = [0.5 * (z_plates_2[i] + z_plates_2[i+1]) for i in range(num_plates - 1)]

# Z coordinates for Module 3
z_start_3 = z_start_2 + H_gap + t_disk # 0.2002
z_plates_3 = [z_start_3 + 0.5 * t_spring + i * (t_spring + gap_spring) for i in range(num_plates)]
z_curves_3 = [0.5 * (z_plates_3[i] + z_plates_3[i+1]) for i in range(num_plates - 1)]

# Step size for the bodies between disk centers (4 joints per module)
dz = (H_gap + t_disk) / 4  # 0.0184
hz_joint = -0.5 * dz  # Joint position relative to child body center

# Z centers of all 13 bodies (global z)
body_z_centers = {
    0: 0.0500,        # Base disk
    1: 0.0500 + dz,   # Seg 1-1
    2: 0.0500 + 2*dz, # Seg 1-2
    3: 0.0500 + 3*dz, # Seg 1-3
    4: 0.1236,        # Mid disk 1
    5: 0.1236 + dz,   # Seg 2-1
    6: 0.1236 + 2*dz, # Seg 2-2
    7: 0.1236 + 3*dz, # Seg 2-3
    8: 0.1972,        # Mid disk 2
    9: 0.1972 + dz,   # Seg 3-1
    10: 0.1972 + 2*dz,# Seg 3-2
    11: 0.1972 + 3*dz,# Seg 3-3
    12: 0.2708,       # Top disk
}

def generate_hollow_disk_stl(filename, R_out, R_in, thickness, N=32):
    h = thickness / 2.0
    vertices = []
    # Top outer
    for i in range(N):
        theta = i * 2 * math.pi / N
        vertices.append((R_out * math.cos(theta), R_out * math.sin(theta), h))
    # Top inner
    for i in range(N):
        theta = i * 2 * math.pi / N
        vertices.append((R_in * math.cos(theta), R_in * math.sin(theta), h))
    # Bottom outer
    for i in range(N):
        theta = i * 2 * math.pi / N
        vertices.append((R_out * math.cos(theta), R_out * math.sin(theta), -h))
    # Bottom inner
    for i in range(N):
        theta = i * 2 * math.pi / N
        vertices.append((R_in * math.cos(theta), R_in * math.sin(theta), -h))
        
    facets = []
    
    def add_quad(v1, v2, v3, v4):
        facets.append((v1, v2, v3))
        facets.append((v1, v3, v4))
        
    # Top face
    for i in range(N):
        next_i = (i + 1) % N
        add_quad(i, next_i, N + next_i, N + i)
        
    # Bottom face
    for i in range(N):
        next_i = (i + 1) % N
        add_quad(2*N + next_i, 2*N + i, 3*N + i, 3*N + next_i)
        
    # Outer wall
    for i in range(N):
        next_i = (i + 1) % N
        add_quad(next_i, i, 2*N + i, 2*N + next_i)
        
    # Inner wall
    for i in range(N):
        next_i = (i + 1) % N
        add_quad(N + i, N + next_i, 3*N + next_i, 3*N + i)
        
    # Write Binary STL
    with open(filename, "wb") as f:
        # 80-byte header
        f.write(b'\x00' * 80)
        # 4-byte number of facets
        f.write(struct.pack("<I", len(facets)))
        
        for f_indices in facets:
            v1 = vertices[f_indices[0]]
            v2 = vertices[f_indices[1]]
            v3 = vertices[f_indices[2]]
            
            ux, uy, uz = v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2]
            vx, vy, vz = v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2]
            nx = uy*vz - uz*vy
            ny = uz*ux - ux*vz
            nz = ux*vy - uy*ux
            length = math.sqrt(nx**2 + ny**2 + nz**2)
            if length > 0:
                nx, ny, nz = nx/length, ny/length, nz/length
            else:
                nx, ny, nz = 0.0, 0.0, 1.0
                
            # Pack normal and vertices: 12 floats + 2 bytes attribute = 50 bytes per facet
            data = struct.pack(
                "<ffffffffffffH",
                nx, ny, nz,
                v1[0], v1[1], v1[2],
                v2[0], v2[1], v2[2],
                v3[0], v3[1], v3[2],
                0
            )
            f.write(data)

def generate_spring_geoms_for_module(body_id, module_id):
    if module_id == 1:
        angles = [0, 90, 180, 270] # E, N, W, S
        z_plates = z_plates_1
        z_curves = z_curves_1
        module_bodies = [0, 1, 2, 3, 4]
    elif module_id == 2:
        angles = [45, 135, 225, 315] # NE, NW, SW, SE
        z_plates = z_plates_2
        z_curves = z_curves_2
        module_bodies = [4, 5, 6, 7, 8]
    else:
        angles = [0, 90, 180, 270] # E, N, W, S
        z_plates = z_plates_3
        z_curves = z_curves_3
        module_bodies = [8, 9, 10, 11, 12]
        
    z_center = body_z_centers[body_id]
    xml_lines = []
    
    body_plates = []
    for i, z_g in enumerate(z_plates):
        best_b = min(module_bodies, key=lambda b: abs(z_g - body_z_centers[b]))
        if best_b == body_id:
            body_plates.append(i)
            
    body_curves = []
    for i, z_g in enumerate(z_curves):
        best_b = min(module_bodies, key=lambda b: abs(z_g - body_z_centers[b]))
        if best_b == body_id:
            body_curves.append(i)
            
    for theta in angles:
        rad = math.radians(theta)
        cos_t = math.cos(rad)
        sin_t = math.sin(rad)
        
        # Plates
        for idx in body_plates:
            z_local = z_plates[idx] - z_center
            x_seg = R_spring * cos_t
            y_seg = R_spring * sin_t
            
            xml_lines.append(
                f'            <geom type="box" pos="{x_seg:.6f} {y_seg:.6f} {z_local:.6f}" '
                f'euler="0 0 {theta}" size="{d_spring:.6f} {0.5 * w_spring:.6f} {0.5 * t_spring:.6f}" rgba="{spring_rgba}"/>'
            )
            
        # Curves (vertical connectors)
        for idx in body_curves:
            z_local = z_curves[idx] - z_center
            if idx % 2 == 0:
                x_local = R_spring + d_spring
            else:
                x_local = R_spring - d_spring
                
            x_seg = x_local * cos_t
            y_seg = x_local * sin_t
            
            xml_lines.append(
                f'            <geom type="box" pos="{x_seg:.6f} {y_seg:.6f} {z_local:.6f}" '
                f'euler="0 0 {theta}" size="{0.5 * t_spring:.6f} {0.5 * w_spring:.6f} {0.5 * gap_spring:.6f}" rgba="{spring_rgba}"/>'
            )
            
    return "\n".join(xml_lines)

def generate_body_xml(body_id):
    # Determine name and child ID
    if body_id == 0:
        name = "disk_1_base"
        pos_str = 'pos="0 0 0.05"'
    elif body_id == 4:
        name = "disk_mid1"
        pos_str = f'pos="0 0 {dz:.6f}"'
    elif body_id == 8:
        name = "disk_mid2"
        pos_str = f'pos="0 0 {dz:.6f}"'
    elif body_id == 12:
        name = "disk_top"
        pos_str = f'pos="0 0 {dz:.6f}"'
    else:
        module_num = (body_id - 1) // 4 + 1
        segment_num = (body_id - 1) % 4 + 1
        name = f"segment_{module_num}_{segment_num}"
        pos_str = f'pos="0 0 {dz:.6f}"'
        
    indent = "    " * (body_id + 2)
    lines = []
    lines.append(f'{indent}<body name="{name}" {pos_str}>')
    
    # Add joints if it's not the base body (body_id > 0)
    if body_id > 0:
        if body_id == 4:
            j_x, j_y = "bend_x_mid1", "bend_y_mid1"
        elif body_id == 8:
            j_x, j_y = "bend_x_mid2", "bend_y_mid2"
        elif body_id == 12:
            j_x, j_y = "bend_x_top", "bend_y_top"
        else:
            module_num = (body_id - 1) // 4 + 1
            segment_num = (body_id - 1) % 4 + 1
            j_x, j_y = f"bend_x_{module_num}_{segment_num}", f"bend_y_{module_num}_{segment_num}"
            
        lines.append(f'{indent}    <joint name="{j_x}" type="hinge" axis="1 0 0" pos="0 0 {hz_joint:.6f}" stiffness="60.0" damping="2.0" group="3"/>')
        lines.append(f'{indent}    <joint name="{j_y}" type="hinge" axis="0 1 0" pos="0 0 {hz_joint:.6f}" stiffness="60.0" damping="2.0" group="3"/>')
        
    # Add hollow circular mesh for disks
    if body_id in [0, 4, 8, 12]:
        if body_id == 0:
            rgba = "0.2 0.2 0.2 1"
        elif body_id == 4:
            rgba = "0.4 0.4 0.4 1"
        elif body_id == 8:
            rgba = "0.6 0.6 0.6 1"
        else:
            rgba = "0.8 0.2 0.2 1" # red top disk
        lines.append(f'{indent}    <geom type="mesh" mesh="hollow_disk" rgba="{rgba}"/>')

    # Add central flexible water pipe segment (15mm diameter / 7.5mm radius)
    lines.append(f'{indent}    <geom type="cylinder" size="0.007500 {0.5 * dz:.6f}" pos="0 0 0" rgba="0.1 0.5 0.8 0.8" contype="0" conaffinity="0"/>')
        
    # Add site hole positions for tendons (NE, NW, SW, SE)
    if body_id == 0:
        s_name = "base"
    elif body_id == 4:
        s_name = "mid1"
    elif body_id == 8:
        s_name = "mid2"
    elif body_id == 12:
        s_name = "top"
    else:
        module_num = (body_id - 1) // 4 + 1
        segment_num = (body_id - 1) % 4 + 1
        s_name = f"{module_num}_{segment_num}"
        
    lines.append(f'{indent}    <site name="site_hole_{s_name}_NE" pos="{xy_tendon:.6f} {xy_tendon:.6f} 0" />')
    lines.append(f'{indent}    <site name="site_hole_{s_name}_NW" pos="-{xy_tendon:.6f} {xy_tendon:.6f} 0" />')
    lines.append(f'{indent}    <site name="site_hole_{s_name}_SW" pos="-{xy_tendon:.6f} -{xy_tendon:.6f} 0" />')
    lines.append(f'{indent}    <site name="site_hole_{s_name}_SE" pos="{xy_tendon:.6f} -{xy_tendon:.6f} 0" />')
    
    # Add spring geoms
    spring_xmls = []
    if body_id == 4:
        spring_xmls.append(generate_spring_geoms_for_module(4, 1))
        spring_xmls.append(generate_spring_geoms_for_module(4, 2))
    elif body_id == 8:
        spring_xmls.append(generate_spring_geoms_for_module(8, 2))
        spring_xmls.append(generate_spring_geoms_for_module(8, 3))
    else:
        if body_id in [0, 1, 2, 3]:
            m = 1
        elif body_id in [5, 6, 7]:
            m = 2
        else:
            m = 3
        spring_xmls.append(generate_spring_geoms_for_module(body_id, m))
        
    for s_xml in spring_xmls:
        if s_xml:
            for l in s_xml.split('\n'):
                lines.append(f'{indent}    {l}')
                
    # Recursively add child body
    if body_id < 12:
        child_xml = generate_body_xml(body_id + 1)
        lines.append(child_xml)
        
    lines.append(f'{indent}</body>')
    return "\n".join(lines)

def generate_tendon_xml():
    suffixes = ["base"]
    for m in [1, 2, 3]:
        for s in [1, 2, 3]:
            suffixes.append(f"{m}_{s}")
        if m < 3:
            suffixes.append(f"mid{m}")
    suffixes.append("top")
    
    xml_lines = []
    for direction in ["NE", "NW", "SW", "SE"]:
        xml_lines.append(f'        <spatial name="cable_{direction}">')
        for s in suffixes:
            xml_lines.append(f'            <site site="site_hole_{s}_{direction}"/>')
        xml_lines.append('        </spatial>')
    return "\n".join(xml_lines)

def build_xml():
    xml = f"""<mujoco model="3_module_12_spring_tdcr">
    <compiler meshdir="."/>
    <option gravity="0 0 -9.81"/>
    
    <visual>
        <scale jointlength="0" jointwidth="0"/>
    </visual>
    
    <asset>
        <mesh name="hollow_disk" file="hollow_disk.stl"/>
    </asset>
    
    <default>
        <!-- Small visible dots for tendon routing holes -->
        <site size="0.0008" rgba="0.2 0.2 0.2 1"/>
        <!-- Thin dark-gray cords for tendons -->
        <tendon width="0.0005" rgba="0.1 0.1 0.1 1"/>
    </default>
    
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
        
        <!-- NESTED BODY TREE -->
{generate_body_xml(0)}
    </worldbody>

    <tendon>
{generate_tendon_xml()}
    </tendon>

    <actuator>
        <motor name="pull_NE" tendon="cable_NE" ctrlrange="0 50"/>
        <motor name="pull_NW" tendon="cable_NW" ctrlrange="0 50"/>
        <motor name="pull_SW" tendon="cable_SW" ctrlrange="0 50"/>
        <motor name="pull_SE" tendon="cable_SE" ctrlrange="0 50"/>
    </actuator>
</mujoco>
"""
    return xml

if __name__ == "__main__":
    # Generate the circular hollow disk STL mesh
    generate_hollow_disk_stl("hollow_disk.stl", R_disk, 0.008000, t_disk, N=64)
    
    # Write the MuJoCo XML
    with open("tdcr_arm.xml", "w") as f:
        f.write(build_xml())
    print("Successfully generated tdcr_arm.xml with hollow circular meshes and 3 modules.")
