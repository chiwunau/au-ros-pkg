#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "end_force_estimation")
(pr2)

(setq *ri* (instance pr2-interface :init))
(ros::rate 10)
(ros::advertise "/end_effector_force" visualization_msgs::MarkerArray 1)
(ros::advertise "/wrench" geometry_msgs::WrenchStamped 1)


(do-until-key
 (ros::spin-once)
 (let ((msg (instance geometry_msgs::WrenchStamped :init))
       (tau (float-vector))
       J force jnt_names jnt_efforts target root tip)
   
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (setq jnt_efforts (send *ri* :state :effort))
   (setq jnt_names (send *ri* :state :name))
   (setq target (send *pr2* :larm :end-coords))
   (setq J (send *pr2* :calc-jacobian-from-link-list
		 (send *pr2* :link-list (send target :parent))
		 :move-target target
		 :rotation-axis t
		 :translation-axis t))

   (let ((ll (send *pr2* :link-list (send target :parent)))
	 jnt_name jnt_eff)
     (mapcar #'(lambda (l)
	       (setq jnt_name (send l :joint :name))
	       (setq jnt_eff (float-vector (elt jnt_efforts (position jnt_name jnt_names :test 'equal))))
	       (setq tau (concatenate float-vector tau jnt_eff)))
	     ll))

   ;(print J)
   ;(setq Jt# (sr-inverse (transpose J)))
   (print tau)
   (setq force (scale -1(transform J tau)))
   (print force)
   
   (send msg :header :stamp (send *ri* :state :stamp))
   (send msg :header :frame_id (send (send target :parent) :name))
   (send msg :wrench :force :x (elt force 0))
   (send msg :wrench :force :y (elt force 1))
   (send msg :wrench :force :z (elt force 2))
   (send msg :wrench :torque :x (elt force 3))
   (send msg :wrench :torque :y (elt force 4))
   (send msg :wrench :torque :z (elt force 5))
   (ros::publish "/wrench" msg)
   ))
			  
 
 