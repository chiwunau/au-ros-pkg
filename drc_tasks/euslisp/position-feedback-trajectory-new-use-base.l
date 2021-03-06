(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://pr2eus/speak.l")
(ros::roseus "pr2_position_feedback_interaction")
(unless (boundp '*irtviewer*) (make-irtviewer))

(defparameter *use-arm* :rarm)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(defparameter *gripper-cds-st* (make-coords))
(defparameter *obj-trans* (make-coords))
(defparameter *base-trans* (make-coords))
(defparameter *obj-gripper-trans* (make-coords))
(defparameter *obj-cds-st* nil)
(defparameter *obj-cds-pre* (make-coords))
(defparameter *fb-inr* 5); n times * 10 mm
(defparameter *t-long* 300) ;msec
(defparameter *t-short* 200) ;msec
(defparameter *t-gap* 50.0) ;msec
(defparameter *ct-gap* 0.0) ;msec
(defparameter *k-gain-f* 0.8)
(defparameter *i-gain-f* 0.8)
;; (defparameter *k-gain-f* 0.00001)
;; (defparameter *i-gain-f* 0.00001)

(setq hoge-av #f(100.414 23.6311 57.2893 -0.408905 -117.46 0.004609 -5.79251 -0.000154 -31.1512 45.5591 -85.6182 -82.7801 37.6429 -5.72958 18.461 -1.29946 -21.2682))

(ros::rate 20)
(setq obj-trans-seq nil)
(setq dif-cds-seq nil)
(setq ang-seq nil)
(setq dif-v-seq nil)
(setq full-av-seq nil)
(setq *base-cds-seq* nil)
(setq *pull* t)
(setq cube-obj-pre nil)

(defun pr2-open-door ()
  (init-param)
  (do-until-key
   (send *irtviewer* :draw-objects :flush t)
   (if (move-object)
       (progn 
	 (print "adjust-base")
	 (speak-jp "移動します")
	 (unless (adjust-base-location)
	   (return-from pr2-open-door))
	 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	 (send *obj-cds-pre* :replace-pos (send (send *pr2* *use-arm* :end-coords :copy-worldcoords) :worldpos))
	 (send *obj-cds-pre* :translate (float-vector (* -1 *fb-inr* 0.5 10) 0 0))
	 (send cube-obj-pre :move-to *obj-cds-pre* :world)
	 )
     (progn
       (print "move object finished")
       (return-from pr2-open-door)))
   )
  )

(defun init-param ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
  (init-obj-cds)
  (send *obj-cds-pre* :move-to (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send *obj-cds-st* :worldrot)) :copy-worldcoords) :world)
  (send *obj-cds-pre* :translate (float-vector (* -1 *fb-inr* 10) 0 0))
  (setq *obj-trans* (make-coords))
  (update-base-trans)
  (setq cds-seq nil)
  (setq ang-seq nil)
  (setq dif-v-seq nil)
  (setq full-av-seq nil)
  (setq dif-cds-seq nil)
  (setq cube-obj-pre (make-cube 40 40 40))
  (send cube-obj-pre :set-color :blue)
  (send cube-obj-pre :move-to *obj-cds-pre* :world)
  (send *irtviewer* :objects (list *pr2* cube-obj-pre)))

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
      (send *irtviewer* :draw-objects :flush t)
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
      (let* ((m (matrix #f(1 0 0) #f(0 1 0) #f(0 0 0)))
	    (ang (vector-angle (transform m (matrix-column (send *obj-cds-st* :worldrot) 0))  (transform m (matrix-column (send *obj-cds-pre* :worldrot) 0)) #f(0 0 1))))
	(send gripper-pos-cds :rotate (deg2rad (if (>= ang 0.0) 10 -10)) :z)));;need edit for both direction
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
    (unless (calc-gripper-trajectory obj-cds (/ *fb-inr* 2))
      (send *pr2* :move-to pr2-cds :world)
      (send *pr2* :angle-vector pr2-av)
      (return-from check-ik-solvable-at-location nil))
    (send *pr2* :move-to pr2-cds :world)
    (send *pr2* :angle-vector pr2-av)
    t))

(defun move-object ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let*  ((obj-cds-st (send *obj-cds-st* :copy-worldcoords))
	  (obj-cds-pre (send *obj-cds-pre* :copy-worldcoords))
	  (obj-cds (make-coords))
	  (obj-trans (send *obj-trans* :copy-worldcoords))
	  (base-cds-pre (send *pr2* :copy-worldcoords))
	  (av-seq nil)
	  (check-time (ros::time+ (ros::time-now) (ros::time 1.0)))
	  ;;check-time
	  res
	  dist-from-pre
	  (fb-inr *fb-inr*);cm
	  (t-long *t-long*) ;msec
	  (t-short *t-short*) ;msec
	  (t-gap *t-gap*)) ;sec

    (send *irtviewer* :objects (send obj-cds :copy-worldcoords))
    (while (ros::ok)
      (when (>= (send (ros::time- (ros::time-now) check-time) :to-sec) 0.0)
	(format t "real check-time:~A~%" (ros::time-now))
	(print "start feedback")
	(setq st-time (ros::time+ (ros::time-now) (ros::time (+ (* fb-inr (/ t-short 1000.0) 0.5) (/ t-gap 1000.0)))))
	(setq check-time (ros::time+ st-time (ros::time (+ (/ t-long 1000.0) (* fb-inr (/ t-short 1000.0) 0.5) (/ *ct-gap* 1000)))))
	
	(format t "set st-tiim:~A check-time:~A~%" st-time check-time)
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(send obj-cds :move-to (send *obj-cds-st* :copy-worldcoords) :world)
	(send obj-cds :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))

	;;check if distance from previous location is too close
	;; (print obj-cds-pre)
	;; (print obj-cds)
	(let (ans
	      (trans (send obj-cds-pre :transformation obj-cds)))
	  (setq dist-from-pre (norm (send trans :worldpos)))
	  (print dist-from-pre)
	  (when (or (< dist-from-pre (/ (* fb-inr 10) 2.5)) (< (elt (send trans :worldpos) 0) 0.0))
	    (format t "object is stuck, dist-from-pre:~A~%" dist-from-pre)
	    (while (not (or (equal ans 'yes) (equal ans 'no)))
	      (print "continute?(yes/no)")
	      (setq ans (read))
	    (if (equal ans 'yes)
		(return-from move-object t)
	      (return-from move-object nil)))))
		
	(send obj-trans :transform (calc-trans-from-pos-error obj-cds-pre obj-cds))
	(push (send obj-trans :copy-worldcoords) obj-trans-seq)
	
	(send obj-cds :transform obj-trans)
	(let ((cube (make-cube 30 30 30)))
	  (send cube :set-color :red)
	  (send cube :move-to (send obj-cds :copy-worldcoords))
	  (send *irtviewer* :objects cube (send obj-cds :copy-worldcoords)))
	(send *irtviewer* :draw-objects :flush t)


	;guess next start point and try to solve ik ;;need to make better
	(let ((next-cds (send obj-cds :copy-worldcoords)))
	  ;;(print (+ (- (* fb-inr 10) dist-from-pre) (* fb-inr 10 0.5)))
	  ;; (send obj-cds :translate (float-vector (* dist-from-pre 0.5) 0 0))
	  (send next-cds :translate (float-vector (* fb-inr 10 0.5) 0 0))
	  ;; (send obj-cds :translate (float-vector (+  (- (* fb-inr 10) dist-from-pre) (* fb-inr 10 0.5)) 0 0))
	  (send base-cds-pre :move-to (send *pr2* :copy-worldcoords) :world)
	  (setq av-seq (calc-gripper-trajectory next-cds *fb-inr*)))
	;;move base if gripper is too close to body
	(let* ((c (send *pr2* *use-arm* :end-coords :worldpos))
	      (d (norm (v- (float-vector (elt c 0) (elt c 1) 0.0) (send *pr2* :worldpos)))))
	  (unless (and (> d 400) av-seq)
	    (unless (> d 400)
	      (print "gripper is too close to body"))
	    (unless av-seq 
	      (print "failed to solve ik"))
	    (print " try to move base")
	    (send *pr2* :move-to base-cds-pre :world)
	    (return-from move-object t)))

	;;else robot start moving
	(send obj-cds-pre :move-to (send obj-cds :copy-worldcoords) :world)
	(send *obj-cds-pre* :move-to (send obj-cds-pre :copy-worldcoords) :world)
	(send cube-obj-pre :move-to *obj-cds-pre* :world)

	(send *ri* :angle-vector-sequence av-seq (list t-long t-short)  :default-controller st-time)
	(let* ((base-recent (send base-cds-pre :copy-worldcoords))
	       (base-tgt (send *pr2* :copy-worldcoords))
	       (diff-cds (send base-recent :transformation base-tgt))
	       (dif-v (send diff-cds :pos))
	       (move-time (+ t-long (* (- (length av-seq) 1) t-short)))
	       (sttm st-time)
	       )
	  (send *pr2* :move-to base-tgt :world)
	  (print move-time)
	  (print sttm)
	  (send *ri* :move-trajectory
		(/ (elt dif-v 0) move-time) (/ (elt dif-v 1) move-time) (/ (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1)) (/ move-time 1000.0))
		move-time :start-time sttm :send-action t))
	  (push av-seq full-av-seq)
	  (send *obj-trans* :move-to (send obj-trans :copy-worldcoords) :world))
	  (ros::sleep)
	  )))

(defun calc-gripper-trajectory (tgt-cds d) ;start-coords distance[cm]
  (let* ((obj-cds (make-cascoords :coords (send tgt-cds :copy-worldcoords)))
	 (c (send (send obj-cds :copy-worldcoords) :transform *obj-gripper-trans*))
	 (gripper-cds (make-cascoords :coords c))
	av-seq)
    
    ;;(send *irtviewer* :objects obj-cds)
    (send obj-cds :assoc gripper-cds)
    (dotimes (i (+ d 1))
      (send *irtviewer* :draw-objects :flush t)
      (unless (send *pr2* :inverse-kinematics gripper-cds 
		    :move-target (send *pr2* *use-arm* :end-coords)
		    :link-list (send *pr2* :link-list (send *pr2* *use-arm* :end-coords :parent))
		    :use-torso nil
		    :use-base t		    
		    :rotational-axis t)
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
	    (/ (elt dif-v 0) move-time) (/ (elt dif-v 1) move-time) (/ (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1)) (/ move-time 1000.0))
	    move-time :start-time sttm :send-action t)
      )))

(defun calc-trans-from-pos-error (pre-cds cds)
  (let* ((diff-cds (make-coords))
	 (output-trans (make-coords))
	 (dif-v (send (send pre-cds :transformation cds) :pos))
	 (v-proj (float-vector (elt dif-v 0) (elt dif-v 1) 0.0))
	 (ang-z (vector-angle #f(1 0 0) v-proj #f(0 0 1)))
	 ang-y
	 (dist-gain (/(norm dif-v) (* *fb-inr* 10)))
	 (k-gain (* 0.45 *k-gain-f*))
	 (i-gain (* (/ (+ *t-long* (* *fb-inr* *t-short*) *t-gap*) 1000.0) *k-gain-f*))
	 p-factor
	 i-factor)

	  ;;(setq dist-from-pre (norm (v- (send obj-cds-pre :worldpos) (send obj-cds :worldpos))))
    (send diff-cds :rotate ang-z :z)
    (setq dif-v (send diff-cds :inverse-transform-vector dif-v))
    (setq ang-y (vector-angle #f(1 0 0) dif-v #f(0 1 0)))
    (send diff-cds :rotate ang-y :y)
    (push dif-v dif-v-seq)
    (push diff-cds dif-cds-seq)
    (push (float-vector ang-z ang-y) ang-seq)

    ;; (setq k-gain 0.5)
    ;; (setq i-gain 1000000)
    (format t "dist-gain:=~A~%" dist-gain)
    (setq dist-gain (if (> dist-gain 1.0) 1.0 dist-gain))
    (format t "dist-gain:=~A~%" dist-gain)
    (setq p-factor (car ang-seq))
    (let ((sum #f(0 0)))
      (dolist (ang ang-seq)
	(setq sum (v+ sum ang)))
      (print sum)
      (setq i-factor (scale (/ (+ *t-long* (* *fb-inr* *t-short*) *t-gap*) 1000.0) sum)))
    
    (send output-trans :rotate (* dist-gain k-gain (+ (elt p-factor 0) (/ (elt i-factor 0) i-gain))) :z)
    (send output-trans :rotate (* dist-gain k-gain (+ (elt p-factor 1) (/ (elt i-factor 1) i-gain))) :y)
    ;; diff-cds))
     output-trans))
    





