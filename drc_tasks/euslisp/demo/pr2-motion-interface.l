(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(pr2-init)
(load "package://drc_tasks/euslisp/demo/grasp-obstacle-plane.l")
(load "package://drc_tasks/euslisp/demo/grasp-object.l")
(load "package://drc_tasks/euslisp/demo/manipulate-object.l")
(ros::load-ros-manifest "drc_tasks")
(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus "pr2_grasp_plane_node")

(send *ri* :set-moveit-environment (instance moveit-environment :init))

(unless (boundp '*irtviewer*)
  (make-irtviewer))
(send *irtviewer* :objects *pr2*)

(setq *request-time* (ros::time))
(setq *manip-obj* (instance manipulate-object :init))
(setq *handle-obj* (instance handle-object :init))
(setq *is-busy* nil)
(setq *use-arm* :larm)

(defun menu-click-cb (msg)
  (setq *request-time* (send msg :header :stamp))
  (let ((order (send msg :data)))
    (case order
      (0
       (pr2-grasp-plane))
      (1
       (pr2-release-plane))
      (3
       (setq *use-arm* :larm)
       (send *handle-obj* :grasp-selected-object *use-arm*))
      (4
       (setq *use-arm* :rarm)
       (send *handle-obj* :grasp-selected-object *use-arm*))
      (5
	 (ros::spin-once)
	   (send *manip-obj* :manipulate-object *use-arm*)
	   (send *handle-obj* :release-object *use-arm*)
	   )
      (6
       (send *handle-obj* :release-object *use-arm*)
	 ))))


(ros::subscribe "/ground_object_segmentation/bounding_box_marker/menu_action_request"
		jsk_pcl_ros::Int32Stamped
		#'menu-click-cb)

(defun main ()
  (ros::rate 5)
  (do-until-key
   (ros::spin-once)
   ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   ;; (send *irtviewer* :objects *pr2*)
   ;; (send *irtviewer* :draw-objects :flush t)
   (ros::sleep)
   ))
    
	
	
  

