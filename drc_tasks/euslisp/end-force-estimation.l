#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "end_force_estimation")
(pr2-init)
(ros::advertise "/Left_endeffector_force1" geometry_msgs::WrenchStamped 1)
(ros::advertise "/Right_endeffector_force1" geometry_msgs::WrenchStamped 1)
(ros::rate 10)


(setq t-const 0.2)
(setq factor (exp (/ -1.0 t-const)))

(let ((msg (instance geometry_msgs::WrenchStamped :init))
      (force-pre0 #f(0 0 0 0 0 0)) (force-pre1 #f(0 0 0 0 0 0)) force target)
  (do-until-key
   (dotimes (i 2)
     (if (= i 0) (setq arm :larm)
       (setq arm :rarm))
     (setq target (send *pr2* arm :end-coords))
     
     (setq force (scale -1 (send *pr2* :calc-force-from-joint-torque arm (send *ri* :state :torque-vector) :use-torso nil)))
     (if (= i 0)
	 (setq force (v+ (scale factor force-pre0) (scale (- 1.0 factor) force)))
       (setq force (v+ (scale factor force-pre1) (scale (- 1.0 factor) force))))
     
     (send msg :header :stamp (send *ri* :state :stamp))
     (send msg :header :frame_id (send (send target :parent) :name))
     (send msg :wrench :force :x (elt force 0))
     (send msg :wrench :force :y (elt force 1))
     (send msg :wrench :force :z (elt force 2))
     (send msg :wrench :torque :x (elt force 3))
     (send msg :wrench :torque :y (elt force 4))
     (send msg :wrench :torque :z (elt force 5))
     (if (= i 0) 
	 (progn
	   (ros::publish "/Left_endeffector_force1" msg)
	   (setq force-pre0 force))
       (progn
	 (ros::publish "/Right_endeffector_force1" msg)
	 (setq force-pre1 force))))
   (ros::spin-once)
  ))
 
 