(defun pr2-open-door ()
  (init-param)
  (do-until-key
   (send *irtviewer* :draw-objects :flush t)
   (move-object)
   (print "adjust-base")
   (unless (adjust-base-location)
     (return))))

(defun init-param ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *pr2-gripper-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
  ;;(init-obj-cds)
  (setq cds-seq nil)
  (setq ang-seq nil)
  (setq dif-v-seq nil)
  (setq full-av-seq nil)
  (setq dif-cds-seq nil))


(defun adjust-base-location ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((base-cds (send *pr2* :copy-worldcoords))
	 (gripper-pos-cds (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base :worldrot)))
	 (base-trans (send *base-trans* :copy-worldcoords))
	 res)

    (dotimes (i 10)
      (push base-cds base-cds-seq)
      (setq base-cds (make-coords 
		      :pos (float-vector 
			    (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 0)
			    (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 1)
			    0.0)
		      :rot (send gripper-pos-cds :worldrot)))
      (setq res (check-ik-solvable-at base-cds))
	    (if res (return))
	    (send gripper-pos-cds :rotate (deg2rad -10) :z))

      (if res
	  (progn
	    (hold-arm-move-base base-cds)
	    (send *ri* :wait-interpolation)
	    (update-base-trans)
	    (return-from adjust-base-location t))
	(progn
	  (ros::ros-warn "ik-check filed at all candidate location")
	  (return-from adjust-base-location nil)))
      ))

(defun check-ik-solvable-at (tgt-cds)
  (let* ((base-tgt (send tgt-cds :copy-worldcoords))
	 (pr2-cds (send *pr2* :copy-worldcoords))
	 (pr2-av (send *pr2* :angle-vector))
	 (gripper-tgt (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
    (send *pr2* :move-to base-tgt :world)
    
    (dotimes (i 5)
      (unless (send *pr2* *use-arm* :inverse-kinematics gripper-tgt)
	(return-from check-ik-solvable nil))
      (send gripper-tgt :translate (float-vector (* (if *pull* -1 1) 20) 0 0)))
    (send *pr2* :move-to pr2-cds :world)
    (send *pr2* :angle-vector pr2-av)
    t))

(defun move-object ()
  (let*  ((obj-cds-st (send *obj-cds-st* :copy-worldcoords))
	  (obj-cds-pre (send obj-cds-st :copy-worldcoords))
	  (obj-cds (make-cascoords :coords (send obj-cds-pre :copy-worldcoords)))
	  (gripper-cds (make-cascoords :coords (send cds-st :copy-worldcoords)))
	  (obj-trans *obj-trans*)
	  (av-seq nil)
	  (st-time (ros::time+ (ros::time-now) (ros::time 1.0)))
	  check-time
	  res)
    
    (send obj-cds :assoc gripper-cds)
    ;;move one time when starting
    (dotimes (i 10)
      (send *pr2* *use-arm* :inverse-kinematics gripper-cds :rotational-axis t)
      (push (send *pr2* :angle-vector) av-seq)
      (send obj-cds-st :translate (float-vector 10 0 0)))
    (send *ri* :angle-vector-sequence (reverse av-seq) (list 500 200)  :default-controller st-time)
    (push av-seq full-av-seq)
    (setq check-time (ros::time+ st-time (ros::time 1.4)))
    
    (while (ros::ok)
      (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0.0)
	(setq st-time (ros::time+ (ros::time-now) (ros::time 2.35)))
	(setq check-time (ros::time+ st-time (ros::time 1.4)))
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(send obj-cds :replace-coords (send *pr2* *use-arm* :end-coords :worldpos))
	
	(send obj-trans :transform (calc-trans-from-pos-error obj-cds-pre obj-cds))
	(push (send obj-trans :copy-worldcoords) obj-trans-seq)

	
	(let ((c (make-coords :coords (send obj-st :copy-worldcoords))))
	  (send c :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))
	  (send obj-cds :move-to c :world))
	(send obj-cds :transform obj-trans)

	(setq obj-cds-pre (send obj-cds :copy-worldcoords))
	
	(send obj-cds :translate (float-vector 10 0 0))
	(setq av-seq nil)
	(dotimes (i 10)
	  (unless (send *pr2* *use-arm* :inverse-kinematics gripper-cds :rotational-axis t)
	    (print "solve-ik-failed")
	    (return-from move-obj))
	  (push (send *pr2* :angle-vector) av-seq)
	  (send cds-recent :translate (float-vector 10 0 0)))

	(send *ri* :angle-vector-sequence (reverse av-seq) (list 500 200) :default-controller st-time)
	(push av-seq full-av-seq)
	)

      (ros::sleep)
      )))

(defun calc-gripper-trajectory ()
  (let ((obj-cds (make-cascoords :coords (send *obj-cds* :copy-worldcoords))
	(gripper-cds)
	)
    (send obj-cds :assoc gripper-cds)
    (dotimes (i 10)
      (send *pr2* *use-arm* :inverse-kinematics gripper-cds :rotational-axis t)
      (push (send *pr2* :angle-vector) av-seq)
      (send obj-cds-st :translate (float-vector 10 0 0)))
    av-seq))
  

(defun update-base-trans ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((base-cds (send *pr2* :copy-worldcoords))
	 (gripper-pos-cds (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base-cds :worldrot))))
    (setq *base-trans* (send gripper-pos-cds :transformation base-cds))))

(defun hold-arm-move-base (tgt-cds)
  (let* ((base-recent (send *pr2* :copy-worldcoords))
	 (base-tgt (send tgt-cds :copy-worldcoords))
	 (diff-cds (send base-recent :transformation base-tgt))
	 (dif-v (send diff-cds :pos))
	 (gripper-cds (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	 (move-time 3000)
	 )

    (send *pr2* :move-to base-tgt :world)
    (send *pr2* *use-arm* :inverse-kinematics gripper-cds)
    (let ((sttm (ros::time+ (ros::time-now) (ros::time 0.5))))
      (send *ri* :angle-vector (send *pr2* :angle-vector)
	    move-time :default-controller sttm)
      (send *ri* :move-trajectory
	    (/ (elt dif-v 0) move-time) (/ (elt dif-v 1)) (vector-angle #f(1 0 0) (matrix-row (send diff-cds :rot) 0) #f(0 0 1))
	    move-time :start-time sttm :send-action t)
      )))

(defun calc-trans-from-pos-error (pre-cds cds)
  (let* ((diff-cds (make-coords))
	 (dif-v (send (send pre-cds :transformation cds) :pos))
	 (v-proj (float-vector (elt dif-v 0) (elt dif-v 1) 0.0))
	 (ang-z (vector-angle #f(1 0 0) v-proj #f(0 0 1)))
	 ang-y)
    (cond ((>= ang-z (/ Pi 2.0))
	   (setq *pull* t)
	   (send diff-cds :rotate (+ (* -1 Pi) ang-z) :z))
	  ((<= ang-z (/ Pi -2.0))
	   (setq *pull* t)
	   (send diff-cds :rotate (+ Pi ang-z) :z))
	  ((and (>= ang-z 0) (< ang-z (/ Pi 2.0)))
	   (setq *pull* nil)
	   (send diff-cds :rotate (* (if *pull* 1 -1) ang-z) :z))
	  ((and (< ang-z 0) (> ang-z (/ Pi -2.0)))
	   (setq *pull* nil)
	   (send diff-cds :rotate (* (if *pull* -1 1) ang-z) :z))
	  (t
	   ))
    (setq dif-v (send diff-cds :inverse-transform-vector dif-v))
    (setq ang-y (vector-angle #f(1 0 0) dif-v #f(0 1 0)))

    (cond ((>= ang-y (/ Pi 2.0))
	   (send diff-cds :rotate (+ (* -1 Pi) ang-y) :y))
	  ((<= ang-y (/ Pi -2.0))
	   (send diff-cds :rotate (+ Pi ang-y) :y))
	  ((and (>= ang-y 0) (< ang-y (/ Pi 2.0)))
	   (send diff-cds :rotate ang-y :y))
	  ((and (< ang-y 0) (> ang-y (/ Pi -2.0)))
	   (send diff-cds :rotate (* -1 ang-y) :y))
	  (t
	   ))
    (push dif-v dif-v-seq)
    (push diff-cds dif-cds-seq)
    (push (list ang-z ang-y) ang-seq)
    diff-cds
    ))





