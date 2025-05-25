#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, PlanningOptions
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from rcl_interfaces.srv import GetParameters
import time

def detect_planning_groups():
    """Try to detect available planning groups"""
    rclpy.init()
    node = Node('detect_groups_node')
    
    # Try to get planning groups from parameters
    client = node.create_client(GetParameters, '/move_group/get_parameters')
    
    if client.wait_for_service(timeout_sec=5.0):
        request = GetParameters.Request()
        # Try different parameter names that might contain group info
        possible_params = [
            'robot_description_planning.group_names',
            'planning_groups',
            'group_names',
            'move_group.planning_groups'
        ]
        
        for param_name in possible_params:
            request.names = [param_name]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            
            try:
                result = future.result()
                if result.values and result.values[0].string_array_value:
                    groups = result.values[0].string_array_value
                    node.get_logger().info(f'Found planning groups: {groups}')
                    node.destroy_node()
                    rclpy.shutdown()
                    return groups
            except:
                continue
    
    node.destroy_node()
    rclpy.shutdown()
    
    # If we can't detect automatically, return common names based on your controllers
    node.get_logger().info('Auto-detection failed, trying common names')
    return ['arm_group', 'arm', 'manipulator', 'hand']

def test_planning_group(group_name, x=0.3, y=0.2, z=0.5):
    """Test if a planning group works"""
    rclpy.init()
    node = Node('test_group_node')
    action_client = ActionClient(node, MoveGroup, '/move_action')
    
    node.get_logger().info(f'Testing planning group: {group_name}')
    
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('move_action server not available!')
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    # Create a simple goal
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = group_name
    goal_msg.request.num_planning_attempts = 3
    
    # Simple position constraint
    constraints = Constraints()
    pos_constraint = PositionConstraint()
    pos_constraint.header.frame_id = "base_link"
    pos_constraint.link_name = "link_5"  # We'll try this first
    pos_constraint.target_point_offset.x = x
    pos_constraint.target_point_offset.y = y
    pos_constraint.target_point_offset.z = z
    pos_constraint.weight = 1.0
    
    # Large tolerance
    constraint_region = SolidPrimitive()
    constraint_region.type = SolidPrimitive.BOX
    constraint_region.dimensions = [0.1, 0.1, 0.1]  # 10cm tolerance
    pos_constraint.constraint_region.primitives = [constraint_region]
    pos_constraint.constraint_region.primitive_poses = [PoseStamped().pose]
    
    constraints.position_constraints = [pos_constraint]
    goal_msg.request.goal_constraints = [constraints]
    
    # Plan only (don't execute) for testing
    goal_msg.planning_options.plan_only = True
    
    try:
        # Send goal
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=3.0)
        
        goal_handle = send_goal_future.result()
        if goal_handle and goal_handle.accepted:
            node.get_logger().info(f'✅ Planning group "{group_name}" works!')
            node.destroy_node()
            rclpy.shutdown()
            return True
        else:
            node.get_logger().info(f'❌ Planning group "{group_name}" rejected')
            
    except Exception as e:
        node.get_logger().info(f'❌ Planning group "{group_name}" failed: {str(e)}')
    
    node.destroy_node()
    rclpy.shutdown()
    return False

def move_link5_with_group(planning_group, x, y, z):
    """Move link_5 using the specified planning group"""
    rclpy.init()
    node = Node('move_link5_node')
    action_client = ActionClient(node, MoveGroup, '/move_action')
    
    node.get_logger().info(f'🎯 Moving link_5 to ({x}, {y}, {z}) using group: {planning_group}')
    
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('move_action server not available!')
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    # Create goal
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = planning_group
    goal_msg.request.num_planning_attempts = 10
    
    # Position constraint for link_5
    constraints = Constraints()
    pos_constraint = PositionConstraint()
    pos_constraint.header.frame_id = "base_link"
    pos_constraint.header.stamp = node.get_clock().now().to_msg()
    pos_constraint.link_name = "link_5"
    pos_constraint.target_point_offset.x = x
    pos_constraint.target_point_offset.y = y
    pos_constraint.target_point_offset.z = z
    pos_constraint.weight = 1.0
    
    # Reasonable tolerance
    constraint_region = SolidPrimitive()
    constraint_region.type = SolidPrimitive.BOX
    constraint_region.dimensions = [0.03, 0.03, 0.03]  # 3cm tolerance
    pos_constraint.constraint_region.primitives = [constraint_region]
    pos_constraint.constraint_region.primitive_poses = [PoseStamped().pose]
    
    constraints.position_constraints = [pos_constraint]
    goal_msg.request.goal_constraints = [constraints]
    
    # Planning options - execute the motion
    goal_msg.planning_options.plan_only = False
    goal_msg.planning_options.replan = True
    goal_msg.planning_options.replan_attempts = 3
    
    # Send goal with feedback
    def feedback_callback(feedback_msg):
        node.get_logger().info(f'⚙️  Status: {feedback_msg.feedback.state}')
    
    send_goal_future = action_client.send_goal_async(
        goal_msg, 
        feedback_callback=feedback_callback
    )
    rclpy.spin_until_future_complete(node, send_goal_future)
    
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        node.get_logger().error('❌ Goal rejected!')
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    node.get_logger().info('✅ Goal accepted! Executing motion...')
    
    # Wait for result
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    
    result = get_result_future.result().result
    success = result.error_code.val == 1
    
    if success:
        node.get_logger().info('🎉 Successfully moved link_5!')
    else:
        node.get_logger().error(f'💥 Motion failed with error code: {result.error_code.val}')
    
    node.destroy_node()
    rclpy.shutdown()
    return success

if __name__ == '__main__':
    print("🔍 Auto-detecting planning groups...")
    
    # Try to detect planning groups
    possible_groups = ['arm_group', 'arm', 'manipulator', 'hand']
    
    print(f"🧪 Testing these groups: {possible_groups}")
    
    working_group = None
    for group in possible_groups:
        print(f"\n🔬 Testing group: {group}")
        if test_planning_group(group):
            working_group = group
            print(f"✅ Found working group: {group}")
            break
        time.sleep(1)  # Small delay between tests
    
    if not working_group:
        print("❌ No working planning group found!")
        print("💡 Try checking your MoveIt config files or SRDF")
        exit(1)
    
    # Now use the working group to move link_5
    print(f"\n🚀 Using planning group: {working_group}")
    
    # Target coordinates - CHANGE THESE!
    target_x = 0.214
    target_y = -0.424
    target_z = 0.403
    print(f"🎯 Moving link_5 to: ({target_x}, {target_y}, {target_z})")
    
    success = move_link5_with_group(working_group, target_x, target_y, target_z)
    
    if success:
        print("🎉 Mission accomplished!")
    else:
        print("💥 Mission failed!")
        print("💡 Try adjusting the target coordinates or check if link_5 exists")
