import sys

from map_spatial_wrapper.test2 import main as planner_main

def main():
    task_num = '1'
    type = 'maspatial'
    planner_main(sys.argv[1:], str(task_num), type, f'tasks_jsons/{type}/task{task_num}/planner_steps/')

def talker():
   pub = rospy.Publisher('chatter', String, queue_size=10)
   rospy.init_node('talker', anonymous=True)
   rate = rospy.Rate(10) # 10hz
   while not rospy.is_shutdown():
       hello_str = "hello world %s" % rospy.get_time()
       rospy.loginfo(hello_str)
       pub.publish(hello_str)
       rate.sleep()


if __name__ == '__main__':
    #main()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
