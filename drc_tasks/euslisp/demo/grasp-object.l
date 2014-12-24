(defclass handle-object
  :slots
  (*grasp-obj-tgt-cds*
   *grasp-obj-tgt-pre-cds-array*
   *grasp-obj-tgt-pre-cds*
   *last-update-time*
   ))

(defmethod handle-object
  (:init
   ()
   (setq *grasp-object-tgt-cds* nil)
   (setq *grasp-obj-tgt-pre-cds-array* nil)
   (setq *grasp-obj-tgt-pre-cds* nil)
   (setq *last-update-time* (ros::time))
   (ros::subscribe "/ground_object_segmentation/handle_estimator/output_best"
		   geometry_msgs::PoseStamped
		   #'send self :tgt-pose-cb)
   (ros::subscribe "/ground_object_segmentation/handle_estimator/output_best_preapproach"
		   geometry_msgs::PoseStamped
		   #'send self :tgt-pre-poses-cb))
   


  (:tgt-pose-cb 
   (msg)
   (setq *last-update-time* (send msg :header :stamp))
   (setq *grasp-obj-tgt-cds* (send (ros::tf-pose->coords (send msg :pose)) :translate #f(40 0 0)))
   (send *irtviewer* :objects  *grasp-obj-tgt-cds*)
   (send *irtviewer* :draw-objects :flush t)
)
  
  (:tgt-pre-poses-cb 
   (msg)
   (setq *grasp-obj-tgt-pre-cds* (ros::tf-pose->coords (send msg :pose)))
   (send *irtviewer* :objects  *grasp-obj-tgt-pre-cds*)
   (send *irtviewer* :draw-objects :flush t)
   )
   ;; (let ((poses (send msg :poses)))
   ;;   (setq *grasp-obj-tgt-pre-cds-array* (mapcar #'(lambda (p) (ros::tf-pose->coords p)) poses))
   ;;   (setq *grasp-obj-tgt-pre-cds* (elt *grasp-obj-tgt-pre-cds-array* (/ (length *grasp-obj-tgt-pre-cds-array*) 2)))))
  
  
  (:grasp-selected-object 
   (&optional (use-arm :larm))
   (unless (ros::time= *last-update-time* *request-time*)
     (ros::ros-warn "latest received pose is to old, give up")
     (return-from :grasp-selected-object nil))

     (let ((avs (send self :calc-avs-to-grasp-selected-object use-arm)))
       (when (not (= (length avs) 2))
	 (return-from :grasp-selected-object nil))
       (send *ri* :angle-vector (car avs) 4000)
       (send *ri* :stop-grasp use-arm)
       (send *ri* :wait-interpolation)
       (send *ri* :angle-vector (cadr avs) 1000)
       (send *ri* :wait-interpolation)
       (send *ri* :start-grasp use-arm :gain 0.1)
       (send *ri* :wait-interpolation))
     t)
   
    
    


(:calc-avs-to-grasp-selected-object 
 (&optional (use-arm :larm))
 (let ((tgt-cds (send *grasp-obj-tgt-cds* :copy-worldcoords))
       (pre-cds (send *grasp-obj-tgt-pre-cds* :copy-worldcoords))
       avs)
   ;; (send *irtviewer* :objects tgt-cds pre-cds)
   ;; (send *irtviewer* :draw-objects :flush t)
   (if (send *pr2* :inverse-kinematics
	     pre-cds
	     :move-target (send *pr2* use-arm :end-coords)
	     :link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
	     :use-torso t
	     :rotational-axis t)
       (progn
	 (push (send *pr2* :angle-vector) avs)
	 (if (send *pr2* :inverse-kinematics
	       tgt-cds
	       :move-target (send *pr2* use-arm :end-coords)
	       :link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
	       :use-torso t
	       :rotational-axis t)
	 (push (send *pr2* :angle-vector) avs)))
     (progn
       (if (send *pr2* :inverse-kinematics
	     (send pre-cds :rotate (deg2rad 180) :x)
	     :move-target (send *pr2* use-arm :end-coords)
	     :link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
	     :use-torso t
	     :rotational-axis t)
	   (push (send *pr2* :angle-vector) avs))
       (if (send *pr2* :inverse-kinematics
	     (send tgt-cds :rotate (deg2rad 180) :x)
	     :move-target (send *pr2* use-arm :end-coords)
	     :link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
	     :use-torso t
	     :rotational-axis t)	
       (push (send *pr2* :angle-vector) avs))))
   (if (= (length avs) 2)
       (reverse avs)
     (progn
       (ros::ros-warn "failed to solve ik to grasp object")
       nil))))
)

