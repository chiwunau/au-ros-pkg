(ros::rate 10)


(setq *pr2-gripper-cds* (send *ri* *use-arm* :end-coords :copy-wroldcoords))
(setq *ri-gripper-cds-pre* (send *pr2-gripper-cds* :copy-wroldcoords))

(setq *cds-trans* (make-coords :coords (send *ri-gripper-cds-pre* :transformation *pr2-gripper-cds*)))


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

(let (tgt-cds-seq
      av-seq)
  (while (ros::ok)
    (when (checktime)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (setq cds-trans :transform (calc-transformation-from-error))

      (dotimes (i 5)
	      (let ((tgt send cds-gripper
			 (send *pr2* *use-arm* :inverse-kinematics (
      
    
  
  
  