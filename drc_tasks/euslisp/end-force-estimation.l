#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "end_force_estimation")
(pr2-init)
(ros::advertise "/Left_endeffector_force" geometry_msgs::WrenchStamped 1)
(ros::advertise "/Right_endeffector_force" geometry_msgs::WrenchStamped 1)

(ros::rate 10)
(do-until-key
 (dotimes (i 2)
   (let ((msg (instance geometry_msgs::WrenchStamped :init))
	 force target)
     (if (= i 0) (setq arm :larm)
       (setq arm :rarm))
     (setq target (send *pr2* arm :end-coords))

     (setq force (scale -1 (send *pr2* :calc-force-from-joint-torque arm (send *ri* :state :torque-vector) :use-torso nil)))
     (send msg :header :stamp (send *ri* :state :stamp))
     (send msg :header :frame_id (send (send target :parent) :name))
     (send msg :wrench :force :x (elt force 0))
     (send msg :wrench :force :y (elt force 1))
     (send msg :wrench :force :z (elt force 2))
     (send msg :wrench :torque :x (elt force 3))
     (send msg :wrench :torque :y (elt force 4))
     (send msg :wrench :torque :z (elt force 5))
     (if (= i 0) (ros::publish "/Left_endeffector_force" msg)
       (ros::publish "/Right_endeffector_force" msg))
     ))

 (ros::spin-once)
 )
 
 