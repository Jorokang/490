import json
import random
import os

def parse_map_file(map_file_path):
    """解析 .map 文件，返回地图尺寸和静态障碍物网格。"""
    static_grid = []
    height = 0
    width = 0
    with open(map_file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            if line.startswith("height"):
                height = int(line.split(" ")[1])
            elif line.startswith("width"):
                width = int(line.split(" ")[1])
            elif len(line) > 0 and not line.startswith("type") and not line.startswith("map"):
                static_grid.append(list(line))
    # 验证是否成功读取了width和height，并且grid的尺寸是否匹配
    if height == 0 or width == 0 or len(static_grid) != height or (height > 0 and len(static_grid[0]) != width) :
        # 尝试从 "map" 标签后开始精确读取
        static_grid = [] #重置
        map_lines_started = False
        for line_idx, line_content in enumerate(lines):
            if "map" in line_content.lower() and not map_lines_started:
                map_lines_started = True
                # 提取 height 和 width (假设它们在 "map" 之前)
                for l_idx in range(line_idx):
                    if lines[l_idx].strip().startswith("height"):
                        height = int(lines[l_idx].strip().split(" ")[1])
                    elif lines[l_idx].strip().startswith("width"):
                        width = int(lines[l_idx].strip().split(" ")[1])
                continue
            if map_lines_started and len(line_content.strip()) > 0:
                 static_grid.append(list(line_content.strip()))
        if not (len(static_grid) == height and width > 0 and len(static_grid[0]) == width):
             raise ValueError(f"地图文件 {map_file_path} 格式错误或尺寸不匹配。读取到的 HxW: {len(static_grid)}x{len(static_grid[0]) if static_grid else 0}, 声明的 HxW: {height}x{width}")


    return width, height, static_grid

def parse_json_constraints(json_file_path, base_dir=""):
    """解析 JSON 文件，返回地图文件路径和处理后的动态障碍物。"""
    with open(json_file_path, 'r') as f:
        data = json.load(f)

    # 通常JSON中的map路径是相对于某个基准的，如果不是绝对路径，需要处理
    map_file_info = data['map']
    if not os.path.isabs(map_file_info) and base_dir:
        map_file_info = os.path.join(base_dir, map_file_info)
    
    # data['data'] 是一个列表，我们取第一个元素（根据提供的JSON结构）
    problem_instance = data['data'][0]
    raw_node_constraints = problem_instance.get('node_constraints', {})
    
    return map_file_info, raw_node_constraints

def process_dynamic_obstacles(raw_constraints, width):
    """将原始节点约束转换为更易于查询的格式 {(x,y): [[t_start, t_end], ...]}。"""
    dynamic_obstacles = {}
    if not width or width <= 0: # 防止 width 为0导致的 ZeroDivisionError
        print("警告: 地图宽度无效，无法处理动态障碍。")
        return dynamic_obstacles
        
    for node_id_str, intervals in raw_constraints.items():
        try:
            node_id = int(node_id_str)
            y = node_id // width
            x = node_id % width
            dynamic_obstacles[(x, y)] = intervals
        except ValueError:
            print(f"警告: 无法将节点ID '{node_id_str}' 转换为整数。")
        except ZeroDivisionError:
            print(f"警告: 地图宽度为0，无法计算节点坐标。节点ID: {node_id_str}")
            # 如果发生这种情况，dynamic_obstacles 可能不完整
            # 但如果width在早期检查中有效，则不应在此处发生
            
    return dynamic_obstacles

def is_valid_location(x, y, t, width, height, static_grid, dynamic_obstacles):
    """检查一个时空点 (x, y, t) 是否有效。"""
    # 1. 检查边界
    if not (0 <= x < width and 0 <= y < height):
        return False
    # 2. 检查静态障碍物
    if static_grid[y][x] == '@':
        return False
    # 3. 检查动态障碍物
    if (x, y) in dynamic_obstacles:
        for interval in dynamic_obstacles[(x, y)]:
            if interval[0] <= t <= interval[1]:
                return False
    return True

def generate_random_trajectory(trajectory_id, num_steps, width, height, static_grid, dynamic_obstacles):
    """生成一条随机轨迹。"""
    trajectory = []
    
    # 寻找一个有效的随机起始点 (t=0)
    start_x, start_y = -1, -1
    attempts = 0
    max_attempts = width * height # 最多尝试次数
    while attempts < max_attempts:
        sx = random.randint(0, width - 1)
        sy = random.randint(0, height - 1)
        if is_valid_location(sx, sy, 0, width, height, static_grid, dynamic_obstacles):
            start_x, start_y = sx, sy
            break
        attempts += 1
    
    if start_x == -1:
        print(f"警告: 轨迹 {trajectory_id}: 无法在时间0找到有效的随机起始点。")
        return trajectory # 返回空轨迹

    current_x, current_y = start_x, start_y
    trajectory.append((current_x, current_y, 0))

    for t in range(1, num_steps):
        possible_moves = [
            (0, 0),  # 停留
            (0, 1),  # 下
            (0, -1), # 上
            (1, 0),  # 右
            (-1, 0)  # 左
        ]
        random.shuffle(possible_moves) # 随机化移动尝试的顺序
        
        moved_this_step = False
        for dx, dy in possible_moves:
            next_x, next_y = current_x + dx, current_y + dy
            if is_valid_location(next_x, next_y, t, width, height, static_grid, dynamic_obstacles):
                current_x, current_y = next_x, next_y
                trajectory.append((current_x, current_y, t))
                moved_this_step = True
                break
        
        if not moved_this_step:
            # print(f"轨迹 {trajectory_id}: 在时间步 {t} 从 ({trajectory[-1][0]}, {trajectory[-1][1]}) 卡住，轨迹提前结束。")
            break # 如果没有有效移动，结束当前轨迹的生成

    return trajectory

def save_trajectory_to_file(trajectory, filename):
    """将轨迹保存到文件。"""
    with open(filename, 'w') as f:
        # 可以在这里添加类似 的元数据行，如果需要的话
        # f.write(f"}]\n") 
        for x, y, t in trajectory:
            f.write(f"{x} {y} {t}\n")

# --- 主执行逻辑 ---
if __name__ == "__main__":
    # 用户可配置参数
    json_file_path = '../scens/maze-100-10.json' # 您的JSON文件名
    num_trajectories_to_generate = 5     # 您希望生成的轨迹数量
    num_steps_per_trajectory = 200        # 每条轨迹的最大长度（时间步数）
    output_dir = "../trackers"  # 输出轨迹文件的目录

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    try:
        # 1. 解析JSON文件获取地图路径和原始约束
        # 假设JSON文件和它引用的地图文件在同一目录或相对于脚本的特定结构
        # 如果JSON中的地图路径是相对的，可能需要一个基准目录
        json_base_dir = os.path.dirname(json_file_path) if os.path.dirname(json_file_path) else "."
        map_file_full_path, raw_constraints = parse_json_constraints(json_file_path, base_dir=json_base_dir)
        print(f"地图文件路径: {map_file_full_path}")

        # 2. 解析地图文件
        if not os.path.exists(map_file_full_path):
            # 如果上一步的路径处理不正确，尝试直接使用JSON中的路径（如果它是绝对的或相对于当前工作目录）
            map_file_full_path_alt = json.load(open(json_file_path, 'r'))['map']
            if os.path.exists(map_file_full_path_alt):
                 map_file_full_path = map_file_full_path_alt
            else:
                 # 尝试相对于JSON文件的目录拼接
                 map_file_full_path_alt_rel = os.path.join(os.path.dirname(json_file_path), map_file_full_path_alt)
                 if os.path.exists(map_file_full_path_alt_rel):
                     map_file_full_path = map_file_full_path_alt_rel
                 else:
                     raise FileNotFoundError(f"地图文件未找到: {map_file_full_path} 或 {map_file_full_path_alt} 或 {map_file_full_path_alt_rel}")

        map_width, map_height, static_obstacle_grid = parse_map_file(map_file_full_path)
        print(f"地图尺寸: Width={map_width}, Height={map_height}")

        # 3. 处理动态障碍物
        # 注意：process_dynamic_obstacles 需要地图宽度来转换节点ID
        dynamic_obstacle_lookup = process_dynamic_obstacles(raw_constraints, map_width)
        print(f"处理了 {len(dynamic_obstacle_lookup)} 个节点的动态约束。")

        # 4. 生成并保存轨迹
        for i in range(num_trajectories_to_generate):
            trajectory_id = i + 1
            print(f"\n正在生成轨迹 {trajectory_id}...")
            generated_trajectory = generate_random_trajectory(
                trajectory_id,
                num_steps_per_trajectory,
                map_width,
                map_height,
                static_obstacle_grid,
                dynamic_obstacle_lookup
            )
            
            if generated_trajectory:
                output_filename = os.path.join(output_dir, f"target_trajectory_{trajectory_id}.txt")
                save_trajectory_to_file(generated_trajectory, output_filename)
                print(f"轨迹 {trajectory_id} 已生成并保存到 {output_filename} (长度: {len(generated_trajectory)} 步)")
                # 打印轨迹的前几个点作为示例
                # print("轨迹起始部分:")
                # for point_idx, point in enumerate(generated_trajectory[:5]):
                #     print(f"  {point}")
            else:
                print(f"轨迹 {trajectory_id} 未能成功生成。")

    except FileNotFoundError as e:
        print(f"错误: 文件未找到 - {e}")
    except ValueError as e:
        print(f"错误: 值错误 - {e}")
    except Exception as e:
        print(f"发生了一个未预料的错误: {e}")
        import traceback
        traceback.print_exc()