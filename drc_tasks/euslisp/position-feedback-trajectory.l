(require "package://pr2eus/pr2-interface.l")
(pr2-init)
(ros::roseus "pr2_position_feedback_interaction")
(unless (boundp '*irtviewer*) (make-irtviewer))
(unless (boundp '*use-arm*) (defparameter *use-arm* :larm))

(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(defparameter *pr2-gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
(defparameter *ri-gripper-cds-pre* (send *pr2-gripper-cds-st* :copy-worldcoords))
(defparameter *cds-trans* (make-coords :coords (send *ri-gripper-cds-pre* :transformation *pr2-gripper-cds-st*)))
(defparameter *pull* t)
(defparameter *base-trans* (make-coords))
(defparameter *base-cds-next* (make-coords))


(ros::rate 20)
(setq cds-seq nil)
(setq dif-cds-seq nil)
(setq ang-seq nil)
(setq dif-v-seq nil)
(setq full-av-seq nil)


(defun init-param ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *pr2-gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
  (setq *ri-gripper-cds-pre* (send *pr2-gripper-cds-st* :copy-worldcoords))
  (setq *cds-trans* (make-coords :coords (send *ri-gripper-cds-pre* :transformation *pr2-gripper-cds-st*)))
  (setq cds-seq nil)
  (setq ang-seq nil)
  (setq dif-v-seq nil)
  (setq full-av-seq nil)
  (setq dif-cds-seq nil)
  )

(defun reset-base-trans ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((base-cds (send *pr2* :copy-worldcoords))
	 (gripper-pos-cds (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base-cds :worldrot))))
    (setq *base-trans* (send gripper-pos-cds :transformation base-cds))))


(defun calc-transformation-from-error ()
  (let* ((diff-cds (make-coords))
	 (ri-gripper-cds (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	 (dif-v (send (send *ri-gripper-cds-pre* :transformation ri-gripper-cds) :pos))
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

(setq base-cds-seq nil)
(defun adjust-base-location ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((base-cds-next (send *pr2* :copy-worldcoords))
	(gripper-pos-cds (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base-cds-next :worldrot)))
	(res nil)
	(base-trans (send *base-trans* :copy-worldcoords)))

    (dotimes (i 10)
      (push base-cds-next base-cds-seq)
      (setq base-cds-next (make-coords 
			   :pos (float-vector 
				 (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 0)
				 (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 1)
				 0.0)
			   :rot (send gripper-pos-cds :worldrot)))
      (setq res (check-ik-solvable base-cds-next))
	    (if res (return))
	    (send gripper-pos-cds :rotate (deg2rad -10) :z))

      (if res
	  (progn
	    (setq *base-cds-next* base-cds-next)
	    (move-base-arm)
	    ;(reset-base-trans)
	    (return-from adjust-base-location t))
	(progn
	  (ros::ros-warn "failed to calc next base location")
	  (return-from adjust-base-location nil)))
      )))


(defun check-ik-solvable (tgt-cds)
  (let* ((base-tgt (send tgt-cds :copy-worldcoords))
	 (pr2-cds (send *pr2* :copy-worldcoords))
	 ( pr2-av (send *pr2* :angle-vector))
	 (gripper-tgt (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
    (send *pr2* :move-to base-tgt :world)
    
    (dotimes (i 5)
      (unless (send *pr2* *use-arm* :inverse-kinematics gripper-tgt)
	(return-from check-ik-solvable nil))
      (send gripper-tgt :translate (float-vector (* (if *pull* -1 1) 20) 0 0)))
    (send *pr2* :move-to pr2-cds :world)
    (send *pr2* :angle-vector pr2-av)
    t))





(defun move-base-arm ()
  (let* ((base-recent (send *pr2* :copy-worldcoords))
	 (base-tgt (send *base-cds-next* :copy-worldcoords))
	 (diff-cds (send base-recent :transformation base-tgt))
	 (dif-v (send diff-cds :pos))
	 (gripper-cds (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	 (move-time 2000)
	 )

    (send *pr2* :move-to base-tgt :world)
    (send *pr2* *use-arm* :inverse-kinematics gripper-cds)
    (let ((sttm (ros::time+ (ros::time-now) (ros::time 0.5))))
      (send *ri* :angle-vector (send *pr2* :angle-vector)
	    move-time :default-controller sttm)
      (send *ri* :move-trajectory
	    (/ (elt dif-v 0) move-time) (/ (elt dif-v 1)) (vector-angle #f(1 0 0) (matrix-row (send diff-cds :rot) 0) #f(0 0 1))
	    move-time :start-time sttm :send-action t))))








(defun move-arm-pos-fb ()
  (init-param)
  (let* (tgt-cds-seq
	 (av-seq nil)
	 (cds-st (send *pr2-gripper-cds-st* :copy-worldcoords))
	 (cds-recent (send cds-st :copy-worldcoords))
	 (cds-trans *cds-trans*)
	 (st-time (ros::time+ (ros::time-now) (ros::time 1.0)))
	 check-time
	 res)
    
    ;;move one time when starting
    (dotimes (i 10)
      (send *pr2* *use-arm* :inverse-kinematics cds-recent :rotational-axis t)
      (push (send *pr2* :angle-vector) av-seq)
      (send cds-recent :translate (float-vector (* (if *pull* -1 1) 10) 0 0)))
    (send *ri* :angle-vector-sequence (reverse av-seq) (* 2 200) :default-controller st-time)
    (push av-seq full-av-seq)
    (setq check-time (ros::time+ st-time (ros::time (* 4 0.6))))
    
    (while (ros::ok)
      (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0.0)
	
	(setq st-time (ros::time+ (ros::time-now) (ros::time (* 4 0.65))))
	(setq check-time (ros::time+ st-time (ros::time (* 4 0.6))))
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	
	(send cds-trans :transform (calc-transformation-from-error))
	(push (send cds-trans :copy-worldcoords) cds-seq)


	(setq cds-recent (send cds-st :copy-worldcoords))	
	(send cds-recent :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))
	(send cds-recent :transform cds-trans)

	(setq *ri-gripper-cds-pre* (send cds-recent :copy-worldcoords))
	
	;;(setq cds-recent (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	(send cds-recent :translate (float-vector (* (if *pull* -1 1) (* 2 20)) 0 0))
	

	(setq av-seq nil)
	(dotimes (i 10)
	  (unless (send *pr2* *use-arm* :inverse-kinematics cds-recent :rotational-axis t)
	    (print "ending")
					;(setq *pull* (not *pull*))
	    (return-from move-arm-pos-fb))
	  (push (send *pr2* :angle-vector) av-seq)
	  (send cds-recent :translate (float-vector (* (if *pull* -1 1) 10) 0 0)))

	(send *ri* :angle-vector-sequence (reverse av-seq) (* 2 200) :default-controller st-time)
	(push av-seq full-av-seq)	
	)

      (ros::sleep)
      )))

(defun pr2-open-door ()
  (send *pr2* :move-to (make-coords) :world)
  (do-until-key
   (send *irtviewer* :draw-objects :flush t)
   (reset-base-trans)	   
   (move-arm-pos-fb)
   (print "adjust-base")
   (unless (adjust-base-location)
     (return))))


;; (send *ri* :angle-vector-sequence  (append (reverse (elt full-av-seq 0)) (reverse (elt full-av-seq 1))) 1500 :default-controller 0.5)
  ;; (unix:usleep (* 1000 (+ 500 500)))
  ;; ;;(send *ri* :cancel-angle-vector)
;; (send *ri* :angle-vector-sequence (append (reverse (elt full-av-seq 3)) (reverse (elt full-av-seq 4))) 500 :default-controller)
