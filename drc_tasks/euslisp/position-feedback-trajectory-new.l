;;(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus/speak.l")
(ros::roseus "pr2_position_feedback_interaction")
(unless (boundp '*irtviewer*) (make-irtviewer))

(defparameter *use-arm* :larm)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(defparameter *gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
(defparameter *obj-trans* (make-coords))
(defparameter *base-trans* (make-coords))
(defparameter *obj-gripper-trans* (make-coords))
(defparameter *obj-cds-st* nil)

(ros::rate 20)
(setq obj-trans-seq nil)
(setq dif-cds-seq nil)
(setq ang-seq nil)
(setq dif-v-seq nil)
(setq full-av-seq nil)
(setq *base-cds-seq* nil)

(defun pr2-open-door ()
  (init-param)
  (do-until-key
   (send *irtviewer* :draw-objects :flush t)
   (if (move-object)
       (progn 
	 (print "adjust-base")
	 (speak-jp "移動します")
	 (unless (adjust-base-location)
	   (return-from pr2-open-door)))
     (progn
       (print "move object finished")
       (return-from pr2-open-door)))))

(defun init-param ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
  (init-obj-cds)
  (update-base-trans)
  (setq cds-seq nil)
  (setq ang-seq nil)
  (setq dif-v-seq nil)
  (setq full-av-seq nil)
  (setq dif-cds-seq nil)
  (send *irtviewer* :objects (list *pr2*)))

(defun init-obj-cds ()
  (setq *obj-cds-st* (send (send *pr2* *use-arm* :end-coords :copy-worldcoords) :rotate (deg2rad 0) :z))
  (setq *obj-gripper-trans* (send *obj-cds-st* :transformation (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
  )


(defun adjust-base-location ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((base-cds (send *pr2* :copy-worldcoords))
	 (gripper-pos-cds (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base-cds :worldrot)))
	 (base-trans (send *base-trans* :copy-worldcoords))
	 res)

    (dotimes (i 10)
      (push base-cds *base-cds-seq*)
      (setq base-cds (make-coords 
		      :pos (float-vector 
			    (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 0)
			    (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 1)
			    0.0)
		      :rot (send gripper-pos-cds :worldrot)))
      (setq res (check-ik-solvable-at-location base-cds))
      (send *irtviewer* :draw-objects :flush t)
	    (if res (return))
	    (send gripper-pos-cds :rotate (deg2rad -10) :z));;need edit for both direction
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

(defun check-ik-solvable-at-location (tgt-cds)
  (let* ((base-tgt (send tgt-cds :copy-worldcoords))
	 (pr2-cds (send *pr2* :copy-worldcoords))
	 (pr2-av (send *pr2* :angle-vector))
	 (gripper-tgt (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	 (obj-cds (send *obj-cds-st* :copy-worldcoords)))
    
    (send obj-cds :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))
    (send obj-cds :transform *obj-trans*)
    
    (send *pr2* :move-to base-tgt :world)
    (unless (calc-gripper-trajectory obj-cds 5)
      (return-from check-ik-solvable-at-location nil))
    t))

(defun move-object ()
  (let*  (
	  (obj-cds-pre (send *obj-cds-st* :copy-worldcoords))
	  (obj-cds (make-coords :coords (send obj-cds-pre :copy-worldcoords)))
	  (obj-trans (send *obj-trans* :copy-worldcoords))
	  (av-seq nil)
	  (st-time (ros::time+ (ros::time-now) (ros::time 1.0)))
	  check-time
	  res)

    (send *irtviewer* :objects obj-cds obj-cds-pre)
    ;;move one time when start
    (setq av-seq (calc-gripper-trajectory obj-cds 10))
    (unless av-seq
      (print "failed to solve ik, try to move base")
      (return-from move-object t))
    (send *ri* :angle-vector-sequence av-seq (list 500 200)  :default-controller st-time)
    (push av-seq full-av-seq)
    (setq check-time (ros::time+ st-time (ros::time 1.5)))
    ;;(print av-seq)
    (while (ros::ok)
      (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0.0)
	(print "start feedback")
	(setq st-time (ros::time+ (ros::time-now) (ros::time 2.35)))
	(setq check-time (ros::time+ st-time (ros::time 1.5)))
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(send *irtviewer* :draw-objects :flush t)
	(send obj-cds :move-to (send *obj-cds-st* :copy-worldcoords) :world)
	(send obj-cds :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))

	;;check if distance from previous location is too close
	(let ((dist (norm (v- (send obj-cds-pre :worldpos) (send obj-cds :worldpos))))
	      ans)
	  (print dist)
	  (when (< dist 35.0)
	    (format t "object is stuck, dist:~A~%" dist)
	    (while (not (or (equal ans 'yes) (equal ans 'no)))
	      (print "continute?(yes/no)")
	      (setq ans (read))
	    (if (equal ans 'yes)
		(return-from move-object t)
	      (return-from move-object nil))))
		
	(send *irtviewer* :draw-objects :flush t)
	(send obj-trans :transform (calc-trans-from-pos-error obj-cds-pre obj-cds))
	(push (send obj-trans :copy-worldcoords) obj-trans-seq)
	(send *obj-trans* :move-to (send obj-trans :copy-worldcoords) :world)
	
	(send obj-cds :transform obj-trans)
	(send obj-cds-pre :move-to (send obj-cds :copy-worldcoords) :world)
	
	(send obj-cds :translate #f(30 0 0))
	(setq av-seq (calc-gripper-trajectory obj-cds 10))
	(let ((c (send *pr2* *use-arm* :end-coords :worldpos)))
	  (unless (and 
		   (> (norm (v- (float-vector (elt c 0) (elt c 1) 0.0) (send *pr2* :worldpos))) 500) 
		   av-seq)
	  (print "failed to solve ik, try to move base")
	  (return-from move-object t)))

	(send *ri* :angle-vector-sequence av-seq (list 500 200)  :default-controller st-time)
	(push av-seq full-av-seq))
      (ros::sleep)
      ))))

(defun calc-gripper-trajectory (tgt-cds n)
  (let* ((obj-cds (make-cascoords :coords (send tgt-cds :copy-worldcoords)))
	 (c (send (send obj-cds :copy-worldcoords) :transform *obj-gripper-trans*))
	 (gripper-cds (make-cascoords :coords c))
	av-seq)
    
    (send *irtviewer* :objects obj-cds)
    (send obj-cds :assoc gripper-cds)
    (dotimes (i n)
      (send *irtviewer* :draw-objects :flush t)
      (unless (send *pr2* *use-arm* :inverse-kinematics gripper-cds :rotational-axis t)
	(return-from calc-gripper-trajectory nil))
      (push (send *pr2* :angle-vector) av-seq)
      (send obj-cds :translate #f(10 0 0)))
    (reverse av-seq)))

  

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
	 (move-time 3000.0)
	 )
    
    (send *pr2* :move-to base-tgt :world)
    (send *pr2* *use-arm* :inverse-kinematics gripper-cds)
    (let ((sttm (ros::time+ (ros::time-now) (ros::time 0.5))))
      (send *ri* :angle-vector (send *pr2* :angle-vector)
	    move-time :default-controller sttm)
      (send *ri* :move-trajectory
	    (/ (elt dif-v 0) move-time) (/ (elt dif-v 1) move-time) (vector-angle #f(1 0 0) (matrix-row (send diff-cds :rot) 0) #f(0 0 1))
	    move-time :start-time sttm :send-action t)
      )))

(defun calc-trans-from-pos-error (pre-cds cds)
  (let* ((diff-cds (make-coords))
	 (dif-v (send (send pre-cds :transformation cds) :pos))
	 (v-proj (float-vector (elt dif-v 0) (elt dif-v 1) 0.0))
	 (ang-z (vector-angle #f(1 0 0) v-proj #f(0 0 1)))
	 ang-y)
    ;; (cond ((>= ang-z (/ Pi 2.0))
    ;; 	   (send diff-cds :rotate (+ (* -1 Pi) ang-z) :z))
    ;; 	  ((<= ang-z (/ Pi -2.0))
    ;; 	   (send diff-cds :rotate (+ Pi ang-z) :z))
    ;; 	  ((and (>= ang-z 0) (< ang-z (/ Pi 2.0)))
    ;; 	   (send diff-cds :rotate (* (if *pull* 1 -1) ang-z) :z))
    ;; 	  ((and (< ang-z 0) (> ang-z (/ Pi -2.0)))
    ;; 	   (send diff-cds :rotate (* (if *pull* -1 1) ang-z) :z))
    ;; 	  (t
    ;; 	   ))
    (send diff-cds :rotate ang-z :z)
    (setq dif-v (send diff-cds :inverse-transform-vector dif-v))
    (setq ang-y (vector-angle #f(1 0 0) dif-v #f(0 1 0)))

    ;; (cond ((>= ang-y (/ Pi 2.0)) 
    ;; 	   (send diff-cds :rotate (+ (* -1 Pi) ang-y) :y))
    ;; 	  ((<= ang-y (/ Pi -2.0))
    ;; 	   (send diff-cds :rotate (+ Pi ang-y) :y))
    ;; 	  ((and (>= ang-y 0) (< ang-y (/ Pi 2.0)))
    ;; 	   (send diff-cds :rotate ang-y :y))
    ;; 	  ((and (< ang-y 0) (> ang-y (/ Pi -2.0)))
    ;; 	   (send diff-cds :rotate (* -1 ang-y) :y))
    ;; 	  (t
    ;; 	   ))
    (send diff-cds :rotate ang-y :y)
    (push dif-v dif-v-seq)
    (push diff-cds dif-cds-seq)
    (push (list ang-z ang-y) ang-seq)
    diff-cds))
    





