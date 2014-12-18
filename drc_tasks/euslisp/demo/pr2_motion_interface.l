(require "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://drc_tasks/euslisp/demo/grasp-obstacle-plane.l")
(load "package://drc_tasks/euslisp/demo/grasp-object.l")
(load "package://drc_tasks/euslisp/demo/manipulate-object.l")
(ros::load-ros-manifest "drc_tasks")
(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus "pr2_grasp_plane_node")

(unless (boundp '*irtviewer*)
  (make-irtviewer))

(setq *request-time* (ros::time))
(setq *manip-obj* (instance manipulate-object :init))
(setq *handle-obj* (instance handle-object :init))
(setq *is-busy* nil)

(defun menu-click-cb (msg)
  (setq *request-time* (send msg :header :stamp))
  (let ((order (send msg :data)))
    (case order
      (0
       (pr2-grasp-plane)
       (pr2-release-plane))
      (6
       (let ((use-arm :larm))
	 (ros::spin-once)
	 (send *handle-obj* :grasp-selected-object use-arm)
	 (send *ri* :wait-interpolation)
	 (ros::spin-once)
	 (ros::rate 20)
	 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	 (send *manip-obj* :set-use-arm use-arm)
	 (send *manip-obj* :init-obj-cds)
	 (send *manip-obj* :move-object)
	 (ros::rate 5)
	 )))))


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
    
	
	
  

