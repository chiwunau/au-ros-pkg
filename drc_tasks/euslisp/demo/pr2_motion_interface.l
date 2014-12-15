(require "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://drc_tasks/euslisp/demo/grasp-obstacle-plane.l")
(ros::load-ros-manifest "drc_tasks")
(ros::roseus "pr2_grasp_plane_node")

(unless (boundp '*irtviewer*)
  (make-irtviewer))

(defun menu-click-cb (msg)
  (let ((order (send msg :data)))
    (case order
      (0
       (pr2-grasp-plane)
       (pr2-release-plane))
      (6
       (grasp-selected-object :larm))
    )))

(ros::subscribe "/ground_object_segmentation/bounding_box_marker/menu_action_request"
		std_msgs::Int32
		#'menu-click-cb)

(defun main ()
  (ros::rate 5)
  (do-until-key
   (ros::spin-once)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *irtviewer* :objects *pr2*)
   (send *irtviewer* :draw-objects :flush t)
   (ros::sleep)
   ))
    
	
	
  

(while (ros::ok)
 (ros::spin-once)
 (print (ros::time-now))
 (ros::sleep))
 