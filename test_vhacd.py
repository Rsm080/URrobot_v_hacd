import pybullet as p
import time

def load_my_urdf():
    # 将原来的 p.connect(p.GUI) 替换为：
    p.connect(p.GUI, options="--opengl2")
    p.setGravity(0, 0, -9.81)
    
    # ⚠️ 把这里换成你自己的 URDF 绝对路径！
    my_urdf_path = "/home/thor/Desktop/jsm/VHACD/URDF/ur5_robotiq_140.urdf"
    
    print(f"正在加载: {my_urdf_path}")
    
    try:
        # 如果你的模型包含复杂的网格碰撞，加上 URDF_USE_MESH_COLLISION 标志
        robot_id = p.loadURDF(my_urdf_path, basePosition=[0, 0, 0.5], useFixedBase=False)
        
        print(f"✅ 加载成功！模型 ID 为: {robot_id}")
        
        # 如果你知道你 URDF 里某个零件的名字，想把它的碰撞改掉：
        # 比如把名字叫 "base_link" 的 link 染色
        num_joints = p.getNumJoints(robot_id)
        p.changeVisualShape(robot_id, -1, rgbaColor=[0, 1, 0, 1]) # 底座染绿
        for i in range(num_joints):
            p.changeVisualShape(robot_id, i, rgbaColor=[0, 0, 1, 1]) # 关节染蓝
            
    except Exception as e:
        print(f"❌ 加载失败: {e}")

    # 开启线框，让你看清碰撞网格
    p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
    print("按 'W' 键开关线框。")
    
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    load_my_urdf()