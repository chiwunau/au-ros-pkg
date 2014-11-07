(require "package://pr2eus/pr2-interface.l")

(ros::roseus "pr2_position_feedback_interaction")
(unless (boundp '*irtviewer*) (make-irtviewer))
(unless (boundp '*use-arm*) (defparameter *use-arm* :larm))

(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(defparameter *pr2-gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
(defparameter *ri-gripper-cds-pre* (send *pr2-gripper-cds-st* :copy-worldcoords))
(defparameter *cds-trans* (make-coords :coords (send *ri-gripper-cds-pre* :transformation *pr2-gripper-cds-st*)))
(defparameter *pull* t)
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

(defun start-moving ()

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
    (dotimes (i 5)
      (send *pr2* *use-arm* :inverse-kinematics cds-recent :rotational-axis t)
      (push (send *pr2* :angle-vector) av-seq)
      (send cds-recent :translate (float-vector (* (if *pull* -1 1) 10) 0 0)))
    (send *ri* :angle-vector-sequence (reverse av-seq) (* 2 200) :default-controller st-time)
    (push av-seq full-av-seq)
    (setq check-time (ros::time+ st-time (ros::time (* 2 0.6))))
    
    (while (ros::ok)
      (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0.0)
	
	(setq st-time (ros::time+ (ros::time-now) (ros::time (* 2 0.65))))
	(setq check-time (ros::time+ st-time (ros::time (* 2 0.6))))
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	
	(send cds-trans :transform (calc-transformation-from-error))
	(push (send cds-trans :copy-worldcoords) cds-seq)


	(setq cds-recent (send cds-st :copy-worldcoords))	
	(send cds-recent :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))
	(send cds-recent :transform cds-trans)

	(setq *ri-gripper-cds-pre* (send cds-recent :copy-worldcoords))
	
	;;(setq cds-recent (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	(send cds-recent :translate (float-vector (* (if *pull* -1 1) 20) 0 0))
	

	(setq av-seq nil)
	(dotimes (i 5)
	  (unless (send *pr2* *use-arm* :inverse-kinematics cds-recent :rotational-axis t)
	    (print "ending")
	    (setq *pull* (not *pull*))
	    (return-from start-moving))
	  (push (send *pr2* :angle-vector) av-seq)
	  (send cds-recent :translate (float-vector (* (if *pull* -1 1) 10) 0 0)))

	(send *ri* :angle-vector-sequence (reverse av-seq) (* 2 200) :default-controller st-time)
	(push av-seq full-av-seq)	
	)

      (ros::sleep)
      )))
  

;; (send *ri* :angle-vector-sequence  (append (reverse (elt full-av-seq 0)) (reverse (elt full-av-seq 1))) 1500 :default-controller 0.5)
;; (unix:usleep (* 1000 (+ 500 500)))
;; ;;(send *ri* :cancel-angle-vector)
;; (send *ri* :angle-vector-sequence (append (reverse (elt full-av-seq 3)) (reverse (elt full-av-seq 4))) 500 :default-controller)
