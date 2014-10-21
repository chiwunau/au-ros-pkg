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
         (tau (float-vector))
         arm J Jt# force jnt_names jnt_efforts target)

     (if (= i 0) (setq arm :larm)
       (setq arm :rarm))
     (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
     (setq jnt_efforts (send *ri* :state :effort))
     (setq jnt_names (send *ri* :state :name))
     (setq target (send *pr2* arm :end-coords))
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
     (setq Jt# (sr-inverse (transpose J)))
     (setq force (scale -1 (transform Jt# tau)))

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
 
 