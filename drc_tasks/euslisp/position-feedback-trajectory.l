(ros::rate 10)


(defvar *pr2-gripper-cds-st* (send *ri* *use-arm* :end-coords :copy-worldcoords))
(defvar *ri-gripper-cds-pre* (send *pr2-gripper-cds* :copy-wroldcoords))

(defvar *cds-trans* (make-coords :coords (send *ri-gripper-cds-pre* :transformation *pr2-gripper-cds-st*)))


(defun calc-transformation-from-error ()
  (let* ((diff-cds (make-coords))
	 (ri-gripper-cds (send *pr2* *use-arm* :end-coords))
	 (dif-v (send (send *ri-gripper-cds-pre* :transformation ri-gripper-cds :pos)))
	 (v-proj (float-vector (elt dif-v 0) (elt dif-v 1) 0.0))
	 (ang-z (vector-angle #f(1 0 0) v-proj #f(0 0 1)))
	 ang-y)
    
    (if (> ang-z 0)
	(send diff-cds :rotate (+ (* -1 Pi) ang-z) :z)
      (send diff-cds :rotate (+ Pi ang-z) :z))
    (setq dif-v (send diff-cds :inverse-transform-vector dif-v))
    (setq ang-y (vector-angle #f(1 0 0) dif-v #f(0 1 0)))
    (if (> ang-y 0)
	(send diff-cds :rotate (+ (* -1 Pi) ang-y) :y)
      (send diff-cds :rotate (+ Pi ang-y) :y))
    diff-cds
    ))

(let* (tgt-cds-seq
      av-seq
      (cds-st (send *pr2-gripper-cds-st* :copy-worldcoords))
      cds-recent
      (st-time (ros::time+ (ros::time-now) (ros::time 0.5)))
      (check-time (ros::time+ st-time (ros::time 0.3))))
  
  (while (ros::ok)
    (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (setq cds-trans :transform (calc-transformation-from-error))
      (setq cds-recent (send *pr2-gripper-cds-st* :copy-worldcoords))
      (send cds-recent :replace-pos (send *pr2* *use-arm* :endcoords :worldpos))
      (send cds-recent :transform cds-trans)

      (send cds-recent :translate #f(-10 0 0))
      
      (setq av-seq nil)
      (dotimes (i 5)
	(send cds-recent :translate #f(-10 0 0))
	(send *pr2* *use-arm* :inverse-kinematics cds-recent :rotational-axis t)
	(push (send *pr2* :angle-vector) av-seq))

      (setq *ri-gripper-cds-pre* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
      
      (send *ri* :angle-vector-sequence (reverse av-seq) 100 :default-controller st-time)

      (setq st-time (ros::time+ st-time (ros::time 0.5)))
      (setq check-time (ros::time+ st-time (ros::time 0.3)))
      ))
      
      
      
	

	
    
  
  
  