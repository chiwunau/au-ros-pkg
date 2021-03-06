(ros::load-ros-manifest "roseus")
(ros::roseus "pickupfromtf.l")

(setq *tfl* (instance ros::transform-listener :init))

(defun renew_tgt()
  (let (cds ret)
    (ros::spin-once)
    (setq cds (send *tfl* :lookup-transform "base_link" "tabletop0" (ros::time 0)))
    (unless cds
      (ros::ros-warn ";;lookup-transform failed")
      (return-from renew_tgt nil))

    
    (setq ret (ros::tf-pose->coords (send cds :transform)))
    ret))


(defun move_arm_to_tgt()
  (setq tgt (renew_tgt))
  (send *pr2* :rarm :inverse-kinematics tgt :rotation-axis :z)
)