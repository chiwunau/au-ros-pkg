(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://pr2eus/speak.l")
(ros::roseus "pr2_position_feedback_interaction")
(unless (boundp '*irtviewer*) (make-irtviewer))

(defparameter *use-arm* :rarm)
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
(setq *pull* t)
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
  (setq *base-trans* (make-coords))
  (setq *obj-gripper-trans* (make-coords))

  (init-obj-cds)
  (update-base-trans)
  (setq cds-seq nil)
  (setq ang-seq nil)
  (setq dif-v-seq nil)
  (setq full-av-seq nil)
  (setq dif-cds-seq nil)
  (send *irtviewer* :objects (list *pr2*)))

(defun init-obj-cds ()
  (setq *obj-cds-st* (send (send *pr2* *use-arm* :end-coords :copy-worldcoords) :rotate (deg2rad (if *pull* 180 0)) :z))
  (setq *obj-gripper-trans* (send *obj-cds-st* :transformation (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
  )


(defun adjust-base-location ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((base-cds (send *pr2* :copy-worldcoords))
	 (gripper-pos-cds (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base-cds :worldrot)) :copy-worldcoords))
	 (base-trans (send *base-trans* :copy-worldcoords))
	 res)

    (dotimes (i 10)
      (push base-cds *base-cds-seq*)
      (setq base-cds (make-coords 
		      :pos (float-vector 
			    (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 0)
			    (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 1)
			    0.0)
		      :rot (send gripper-pos-cds :copy-rot)))
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
    (send *pr2* :move-to pr2-cds :world)
    (send *pr2* :angle-vector pr2-av)
    t))

(defun move-object ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let*  ((obj-cds-st (send *obj-cds-st* :copy-worldcoords))
	  (obj-cds-pre (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send obj-cds-st :worldrot)) :copy-worldcoords))
	  obj-cds 
	  (obj-trans (send *obj-trans* :copy-worldcoords))
	  (av-seq nil)
	  (st-time (ros::time+ (ros::time-now) (ros::time 1.0)))
	  check-time
	  res
	  dist-from-pre
	  (fb-inr 10);cm
	  (t-long 500) ;msec
	  (t-short 200) ;msec
	  (t-gap 1.0)) ;sec
    ;;init coords
    (send *irtviewer* :objects obj-cds obj-cds-pre)
    (send obj-cds-pre :transform obj-trans)
    (setq obj-cds (send obj-cds-pre :copy-worldcoords))
    ;;imaginary previous point, so that dist won't be too small at first time
    (send obj-cds-pre :translate (float-vector (* -1 fb-inr 0.5 10) 0 0))

    ;; (print obj-cds-pre)
    ;; (print obj-cds)
    ;;move one time when start
    (setq av-seq (calc-gripper-trajectory obj-cds 10))
    (unless av-seq
      (print "failed to solve ik, try to move base")
      (return-from move-object t)) ;;;need to edit , move base may not work once enter new loop
    (send *ri* :angle-vector-sequence av-seq (list t-long t-short)  :default-controller st-time)
    (push av-seq full-av-seq)
    (setq check-time (ros::time+ st-time (ros::time (+ (/ t-long 1000.0) (* fb-inr (/ t-short 1000.0) 0.5)))))
    ;;(print av-seq)
    (while (ros::ok)
      (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0.0)
	(format t "real check-time:~A~%" (ros::time-now))
	(print "start feedback")
	(setq st-time (ros::time+ (ros::time-now) (ros::time (+ (* fb-inr (/ t-short 1000) 0.5) t-gap))))
	(setq check-time (ros::time+ st-time (ros::time (+ (/ t-long 1000.0) (* fb-inr (/ t-short 1000.0) 0.5)))))
	
	(format t "set st-tiim:~A check-time:~A~%" st-time check-time)
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(send *irtviewer* :draw-objects :flush t)
	(send obj-cds :move-to (send *obj-cds-st* :copy-worldcoords) :world)
	(send obj-cds :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))

	;;check if distance from previous location is too close
	;; (print obj-cds-pre)
	;; (print obj-cds)
	(let (ans)
	  (setq dist-from-pre (norm (v- (send obj-cds-pre :worldpos) (send obj-cds :worldpos))))
	  (print dist-from-pre)
	  (when (< dist-from-pre (/ (* fb-inr 10) 2.5))
	    (format t "object is stuck, dist-from-pre:~A~%" dist-from-pre)
	    (while (not (or (equal ans 'yes) (equal ans 'no)))
	      (print "continute?(yes/no)")
	      (setq ans (read))
	    (if (equal ans 'yes)
		(return-from move-object t)
	      (return-from move-object nil)))))
		
	(send *irtviewer* :draw-objects :flush t)
	(send obj-trans :transform (calc-trans-from-pos-error obj-cds-pre obj-cds))
	(push (send obj-trans :copy-worldcoords) obj-trans-seq)
	
	(send obj-cds :transform obj-trans)
	(send obj-cds-pre :move-to (send obj-cds :copy-worldcoords) :world)
	
	;guess next start point ;;need to make better
	;(print (+ (- (* fb-inr 10) dist-from-pre) (* fb-inr 10 0.5)))
	(send obj-cds :translate (float-vector (* fb-inr 10 0.5) 0 0))
	;; (send obj-cds :translate (float-vector (+  (- (* fb-inr 10) dist-from-pre) (* fb-inr 10 0.5)) 0 0))

	(setq av-seq (calc-gripper-trajectory obj-cds 10))
	;;move base if gripper is too close to body
	(let ((c (send *pr2* *use-arm* :end-coords :worldpos)))
	  (unless (and 
		   (> (norm (v- (float-vector (elt c 0) (elt c 1) 0.0) (send *pr2* :worldpos))) 400) 
		   av-seq)
	  (print "failed to solve ik or gripper is too close, try to move base")
	  (return-from move-object t)))

	(send *ri* :angle-vector-sequence av-seq (list t-long t-short)  :default-controller st-time)
	(push av-seq full-av-seq)
	(send *obj-trans* :move-to (send obj-trans :copy-worldcoords) :world))
      (ros::sleep)
      )))

(defun calc-gripper-trajectory (tgt-cds d) ;start-coords distance[cm]
  (let* ((obj-cds (make-cascoords :coords (send tgt-cds :copy-worldcoords)))
	 (c (send (send obj-cds :copy-worldcoords) :transform *obj-gripper-trans*))
	 (gripper-cds (make-cascoords :coords c))
	av-seq)
    
    (send *irtviewer* :objects obj-cds)
    (send obj-cds :assoc gripper-cds)
    (dotimes (i (+ d 1))
      (send *irtviewer* :draw-objects :flush t)
      (unless (send *pr2* *use-arm* :inverse-kinematics gripper-cds :rotational-axis t)
	(return-from calc-gripper-trajectory nil))
      (push (send *pr2* :angle-vector) av-seq)
      (send obj-cds :translate #f(10 0 0)))
    (reverse av-seq)))

  

(defun update-base-trans ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let* ((base-cds (send *pr2* :copy-worldcoords))
	 (gripper-pos-cds (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base-cds :worldrot)) :copy-worldcoords)))
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
	    (/ (elt dif-v 0) move-time) (/ (elt dif-v 1) move-time) (/ (vector-angle #f(1 0 0) (matrix-row (send diff-cds :rot) 0) #f(0 0 1)) (/ move-time 1000.0))
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
    





