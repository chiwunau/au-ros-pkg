(ros::load-ros-manifest "tabletop_au2014")
(ros::load-ros-manifest "jsk_pcl_ros")
(load "package://pr2eus/pr2-interface.l")
;(load "package://tabletop_au2014/euslisp/pr2-tabletop-demo-temp.l")
(load "package://pr2eus_moveit/tutorials/pr2-tabletop-demo.l")

(ros::roseus "grasp_selected_obj")
(setq *tfl* (instance ros::transform-listener :init))

(pr2)
(unless (boundp '*irtviewer*)
  (make-irtviewer))

(go-to-default-pose)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(objects (list *pr2*))


(setq tgt nil)
(setq obj_cds nil)
(setq cam_cds nil)
(defun set-target(msg)
  (setq obj_cds (ros::tf-pose->coords (send msg :pose)))
  (setq cam_cds (send *tfl* :lookup-transform 
		  "/base_link" 
		  (send msg :header :frame_id)
		  (ros::time 0)))
  (setq tgt (make-coords :pos (v+ (send cam_cds :pos) (transform (send cam_cds :rot) (send obj_cds :pos)))))
  (setq c1 (make-cube 100 100 100))
  (send c1 :locate (send tgt :pos))
  (objects (list *pr2* c1))
  (grasp-object-on-table tgt)
  )
  
(ros::subscribe "bounding_box_marker/selected_box" jsk_pcl_ros::BoundingBox #'set-target)


(defun main()
  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep))
  )