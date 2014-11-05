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
(setq ang-seq nil)
(setq dif-v-seq nil)
(setq full-av-seq nil)


(defun init-param ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *pr2-gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
  (setq *ri-gripper-cds-pre* (send *pr2-gripper-cds-st* :copy-worldcoords))
  (setq *cds-trans* (make-coords :coords (send *ri-gripper-cds-pre* :transformation *pr2-gripper-cds-st*)))
  (setq *pull* t))

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
	   (send diff-cds :rotate ang-z :z))
	  ((and (< ang-z 0) (> ang-z (/ Pi -2.0)))
	   (setq *pull* nil)
	   (send diff-cds :rotate (* -1 ang-z) :z))
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
    (push diff-cds cds-seq)
    (push (list ang-z ang-y) ang-seq)
    diff-cds
    ))

(defun start-moving ()
  (let* (tgt-cds-seq
	 (av-seq nil)
	 (cds-st (send *pr2-gripper-cds-st* :copy-worldcoords))
	 (cds-recent (send cds-st :copy-worldcoords))
	 (cds-trans *cds-trans*)
	 (st-time (ros::time+ (ros::time-now) (ros::time 0.5)))
	 check-time)

    ;;move one time when starting
    (dotimes (i 5)
      (send *pr2* *use-arm* :inverse-kinematics cds-recent :rotational-axis t)
      (push (send *pr2* :angle-vector) av-seq)
      (send cds-recent :translate (float-vector (* (if *pull* -1 1) 10) 0 0)))
    (send *ri* :angle-vector-sequence (reverse av-seq) 200 :default-controller st-time)
    
    (setq check-time (ros::time+ st-time (ros::time 0.6)))
    (setq st-time (ros::time+ st-time (ros::time 1.25)))
    
    (while (ros::ok)
      (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0.0)
	(print "check")
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(send cds-trans :transform (calc-transformation-from-error))
	(setq cds-recent (send cds-st :copy-worldcoords))	
	(send cds-recent :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))
	(send cds-recent :transform cds-trans)
	;;(setq cds-recent (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	(send cds-recent :translate (float-vector (* (if *pull* -1 1) 20) 0 0))
	
	(setq *ri-gripper-cds-pre* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	(setq av-seq nil)
	(dotimes (i 5)
	  (send *pr2* *use-arm* :inverse-kinematics cds-recent :rotational-axis t)
	  (push (send *pr2* :angle-vector) av-seq)
	  (send cds-recent :translate (float-vector (* (if *pull* -1 1) 10) 0 0)))
	(print (length av-seq))
	(send *ri* :angle-vector-sequence (reverse av-seq) 200 :default-controller st-time)
	
	(setq check-time (ros::time+ st-time (ros::time 0.6)))
	(setq st-time (ros::time+ st-time (ros::time 1.25))))
      
      (print *pull*)
      (ros::sleep)
      )))
 
    
    
    
    

    
    
    
  
  