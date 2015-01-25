;; (load "package://pr2eus/pr2-interface.l")
;; (pr2-init)
(require "package://pr2eus/speak.l")
;;(ros::roseus "pr2_position_feedback_interaction")
(unless (boundp '*irtviewer*) (make-irtviewer))

(defclass manipulate-object
  :slots
  (*use-arm*
   *gripper-cds-st*
   *obj-trans*
   *base-trans*
   *obj-gripper-trans*
   *obj-cds-st*
   *obj-cds-pre*
   *fb-inr* ;; n times * 10 mm
   *t-long* ;;msec
   *t-short*;;msec
   *t-gap*  ;;mesc
   *ct-gap*
   *k-gain-f*
   *manipulate-cds*
   cds-seq
   *ang-seq*
   dif-v-seq
   *avs-passed*
   dif-cds-seq
   obj-trans-seq
   *base-cds-seq*
   *last-update-time*
   *speed* ;;mm/s
   *fb-period* ;;msec
   *ik-division*
   ))

(defmethod manipulate-object
  (:init
   (&optional (arm :larm))
   (setq *use-arm* arm)
   (setq *base-trans* (make-coords))
   (setq *obj-gripper-trans* (make-coords))
   (setq *obj-cds-st* nil)
   (setq *obj-cds-pre* (make-coords))
   (setq *fb-inr* 3)
   (setq *speed* 60.0)
   ;;(setq *fb-period* 800.0) ;;for lan network
   (setq *fb-period* 800.0) ;;for wifi network

   (setq *ik-division* 4)
   (setq *ideal-fb-dist* (/ (* *speed* *fb-period*) 1000.0))
   (setq *t-long* 300)
   (setq *t-short* 200)
   (setq *t-gap* 200.0) 
   (setq *ct-gap* 0.0)
   (setq *k-gain-f* 0.8)
   (setq *i-gain-f* 1.2)
   (setq *manipulate-cds* (make-coords))
   (setq *last-update-time* (ros::time))
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *pr2* :move-to (make-coords) :world)
   (setq *gripper-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
   (setq *obj-trans* (make-coords))
   (send self :update-base-trans)
   (send self :init-obj-cds)
   (setq *cube-obj-pre* (make-cube 20 20 20))
   (send *cube-obj-pre* :set-color :blue)
   (send *cube-obj-pre* :move-to *obj-cds-pre* :world)
   (send *irtviewer* :objects *cube-obj-pre*)
   (ros::subscribe "/ground_object_segmentation/bounding_box_marker/manipulate_pose"
		   geometry_msgs::PoseStamped
		   #'send self :manipulate-pose-cb)
   (ros::advertise "/ground_object_segmentation/bounding_box_marker/draw_manipulate_arrow"
		   jsk_pcl_ros::BoundingBox 
		   1)
   )
  
  (:manipulate-pose-cb
   (msg)
   (setq *last-update-time* (send msg :header :stamp))
   (setq *manipulate-cds* (ros::tf-pose->coords (send msg :pose)))
   )

  (:wait-for-manipulate-pose
   ()
   (while (not (ros::time= *last-update-time* *request-time*))
     (ros::ros-info "waiting for arrow click")
     (ros::spin-once)
     (ros::sleep)))

  (:manipulate-object
   (arm)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *pr2* :move-to (make-coords) :world)
   (setq *use-arm* arm)
   (send self :wait-for-manipulate-pose)
   (send self :init-obj-cds)
   (send self :update-base-trans)
   (send *irtviewer* :objects *pr2* *cube-obj-pre*)
   (while t
     (let ((res (send self :move-object)))
       (ros::ros-info (format nil "manipulate object status:~A~%" res))
       (when (or (equal res "rotate-max") (equal res "stuck"))
	 (progn
	   (send self :publish-manipulate-arrow)
	   (let ((t-st (ros::time-now))
		 (wait-t 5.0))
	     (while (not (> (- (send *last-update-time* :to-sec) (send *request-time* :to-sec)) 0.5)) ;;if no further comment, return
	       (when (> (send (ros::time- (ros::time-now) t-st) :to-sec) wait-t)
		 (ros::ros-info "manipulate object finished!")
		 (return-from :manipulate-object t))
	       (print "waiting for input")
	       (ros::spin-once)
	       (ros::sleep)))
	   (ros::ros-info "continute-to-move-object")
	   (send self :init-obj-cds)))
       (when (equal res "move-base")
	 (speak-jp "移動します")
	 (let ((b4-move-pos (send (send *pr2* *use-arm* :end-coords :copy-worldcoords) :worldpos)))
	   (if (send self :adjust-base-location)
	       (progn 
		 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
		 (when (>= (norm (v- b4-move-pos (send (send *pr2* *use-arm* :end-coords :copy-worldcoords) :worldpos))) 100.0)
		   (print (norm (v- b4-move-pos (send (send *pr2* *use-arm* :end-coords :copy-worldcoords) :worldpos))))
		   (return-from :manipulate-object nil))
		 )
	     (progn
	       (ros::ros-warn "failed to move-base")
	       (return-from :manipulate-object nil)))))
       (when (equal res "move-base-closer")
	 (unless (send self :adjust-base-location-closer)
	   (return-from :manipulate-object nil)))
       (when (equal res "move-base-farther")
	 (unless (send self :adjust-base-location-farther)
	   (return-from :manipulate-object nil)))
	  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	  (send *obj-cds-pre* :replace-pos (send (send *pr2* *use-arm* :end-coords :copy-worldcoords) :worldpos))
	  (send *obj-cds-pre* :translate (float-vector (* -1 *ideal-fb-dist*) 0 0))
	  (send *cube-obj-pre* :move-to *obj-cds-pre* :world)
	  )))

  
  (:init-obj-cds 
   ()
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (setq *obj-cds-st* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
   (send *obj-cds-st* :replace-rot (send *manipulate-cds* :copy-rot))
   (setq *obj-gripper-trans* (send *obj-cds-st* :transformation (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
   (send *obj-cds-pre* :move-to (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send *obj-cds-st* :worldrot)) :copy-worldcoords) :world)
   (send *obj-cds-pre* :translate (float-vector (* -1 *ideal-fb-dist*) 0 0))
   ;;(send *obj-cds-pre* :translate (float-vector (* -1 *fb-inr* 10) 0 0))
   (setq *obj-trans* (make-coords))
   (setq *ang-seq* nil)
   )

  (:adjust-base-location 
   ()
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (let* ((cds-now (send *pr2* :copy-worldcoords))
	  (cds-tgt (send cds-now :copy-worldcoords))
	  (gripper-pos-cds (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send cds-tgt :worldrot)) :copy-worldcoords))
	  (base-trans (send *base-trans* :copy-worldcoords))
	  res)
     (dotimes (i 10)
       (send *irtviewer* :draw-objects :flush t)
       (push cds-tgt *base-cds-seq*)
       (setq cds-tgt (make-coords 
		       :pos (float-vector 
			     (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 0)
			     (elt (send (send (send gripper-pos-cds :copy-worldcoords) :transform base-trans) :pos) 1)
			     0.0)
		       :rot (send gripper-pos-cds :copy-rot)))
       (setq res (send self :check-ik-solvable-at-location cds-tgt))
       (send *irtviewer* :draw-objects :flush t)
       (if res (return))
       (let* ((m (matrix #f(1 0 0) #f(0 1 0) #f(0 0 0)))
	      (ang (vector-angle (transform m (matrix-column (send *obj-cds-st* :worldrot) 0))  (transform m (matrix-column (send *obj-cds-pre* :worldrot) 0)) #f(0 0 1))))
	 (send gripper-pos-cds :rotate (deg2rad (if (>= ang 0.0) 10 -10)) :z)));;need edit for both direction
     (if res
	 (progn
	   (send self :hold-arm-move-base cds-tgt)
	   (send *ri* :wait-interpolation)
	   (send self :update-base-trans)
	   (return-from :adjust-base-location t))
       (progn
	 (ros::ros-warn "ik-check filed at all candidate location")
	 (return-from :adjust-base-location nil)))
     ))

  (:adjust-base-location-closer
   ()
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (let* ((cds-now (send *pr2* :copy-worldcoords))
	  (cds-tgt (send cds-now :copy-worldcoords))
	  (gripper-to-base (send (send cds-now :transformation (send *pr2* *use-arm* :end-coords)) :copy-worldcoords))
	  (gripper-vec-to-base (float-vector (elt (send gripper-to-base :worldpos) 0) (elt (send gripper-to-base :worldpos) 0) 0.0))
	  (ang (vector-angle #f(1 0 0) gripper-vec-to-base #f(0 0 1)))
	  res)
     (send cds-tgt :rotate ang :z)
     (send cds-tgt :translate #f(100 0 0))
     (setq res (send self :check-ik-solvable-at-location cds-tgt))
     (if res
	 (progn
	   (send self :hold-arm-move-base cds-tgt)
	   (send *ri* :wait-interpolation)
	   (send self :update-base-trans)
	   (return-from :adjust-base-location-closer t))
       (progn
	 (ros::ros-warn "failed to move closer")
	 (return-from :adjust-base-location-closer nil)))
     ))

  (:adjust-base-location-farther
   ()
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (let* ((cds-now (send *pr2* :copy-worldcoords))
	  (cds-tgt (send cds-now :copy-worldcoords))
	  (gripper-to-base (send (send cds-now :transformation (send *pr2* *use-arm* :end-coords)) :copy-worldcoords))
	  (gripper-vec-to-base (float-vector (elt (send gripper-to-base :worldpos) 0) (elt (send gripper-to-base :worldpos) 0) 0.0))
	  (ang (vector-angle #f(1 0 0) gripper-vec-to-base #f(0 0 1)))
	  res)
     (send cds-tgt :rotate ang :z)
     (send cds-tgt :translate #f(-100 0 0))
     (setq res (send self :check-ik-solvable-at-location cds-tgt))
     (if res
	 (progn
	   (send self :hold-arm-move-base cds-tgt)
	   (send *ri* :wait-interpolation)
	   (send self :update-base-trans)
	   (return-from :adjust-base-location-farther t))
       (progn
	 (ros::ros-warn "failed to move farther")
	 (return-from :adjust-base-location-farther nil)))
     ))
  
  (:check-ik-solvable-at-location 
   (tgt-cds)
   (let* ((base-tgt (send tgt-cds :copy-worldcoords))
	  (pr2-cds (send *pr2* :copy-worldcoords))
	  (pr2-av (send *pr2* :angle-vector))
	  (gripper-tgt (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	  (obj-cds (send *obj-cds-st* :copy-worldcoords)))

     (send obj-cds :replace-pos (send *pr2* *use-arm* :end-coords :worldpos))
     (send obj-cds :transform *obj-trans*)
     (send *pr2* :move-to base-tgt :world)
    (unless (send self :calc-gripper-trajectory obj-cds *ideal-fb-dist* 3)
      (send *pr2* :move-to pr2-cds :world)
      (send *pr2* :angle-vector pr2-av)
      (return-from :check-ik-solvable-at-location nil))
    (send *pr2* :move-to pr2-cds :world)
    (send *pr2* :angle-vector pr2-av)
    t))
  
  (:move-object 
   ()
   ;; (when (> (- (send *request-time* :to-sec) (send *last-update-time* :to-sec)) 0.5)
   ;;   (ros::ros-warn "latest received pose is to old, give up")
   ;;   (return-from :move-object nil))
   (let* ((t-now (ros::time-now))
	  (t-seq (list t-now))
	  (t-check t-now)
	  (t-gap (/ (* 1.5 1000.0 *ideal-fb-dist*) (* *ik-division* *speed*)))
	  avs-pre
	  avs-new
	  tms-pre
	  tms-new
	  st-time-pre
	  (st-time-new (ros::time+ t-now (ros::time (/ *fb-period* 2 1000.0))))
	  dist-from-pre
	  (obj-cds-st (send *obj-cds-st* :copy-worldcoords))
	  (obj-cds-pre (send *obj-cds-pre* :copy-worldcoords))
	  (obj-cds (make-coords))
	  (stuck-interval (round (/ 100 *ideal-fb-dist*)))
	  (stuck-cnt 0)
	  (loop-cnt 0)
	  (stuck-pre-cds (send obj-cds-pre :copy-worldcoords))
	   
	  (obj-trans (send *obj-trans* :copy-worldcoords)))
     
     
     (while (ros::ok)
       (setq t-now (ros::time-now))
       (when (>= (send (ros::time- t-now t-check) :to-nsec) 0)
	 ;(push t-now t-seq)
	 (format t "before:~A~%" t-now)

	 ;;(push (* 1e-6 (send (ros::time- (car t-seq) (cadr t-seq)) :to-nsec)) tms)
	 (push (send *pr2* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t-check)) *avs-passed*)
	 (format t "real joint time error:~A~%" (ros::time-  (car (send *ri* :state :stamp-list)) t-check))
	 (setq t-check (ros::time+ t-check (ros::time (/ *fb-period* 1000.0))))

	 (send obj-cds :move-to (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send *obj-cds-st* :worldrot)) :copy-worldcoords) :world)
	 
	 ;;check if object stuck
	 (if (>= stuck-cnt (1- stuck-interval))
	     (progn
	       (let ((tr (send stuck-pre-cds :transformation obj-cds)))
		 (setq dist-from-pre (norm (send tr :worldpos)))
		 (print dist-from-pre)
		 (when (or (< dist-from-pre (/ (* stuck-interval *ideal-fb-dist*) 3.0)) (< (elt (send tr :worldpos) 0) 0.0))
		   (return-from :move-object "stuck"))
		 (setq stuck-pre-cds (send obj-cds :copy-worldcoords))
		 (setq stuck-cnt 0)))
	   (incf stuck-cnt))
	   
	 (send obj-trans :transform (send self :calc-trans-from-pos-error obj-cds-pre obj-cds))
	 ;;(setq obj-trans (send self :calc-trans-from-pos-error obj-cds-pre obj-cds))
	 (send obj-cds :transform obj-trans)

	 ;;leave when rotate more then 110deg
	 (when (>= (vector-angle (matrix-column (send *obj-cds-st* :worldrot) 0) (matrix-column (send obj-cds :worldrot) 0)) (deg2rad 110))
	   (return-from :move-object "rotate-max"))
			
	   

	 (let ((next-cds (send obj-cds :copy-worldcoords)))
	   ;;(send next-cds :translate (float-vector (+ (- *ideal-fb-dist* dist-from-pre) (/ *ideal-fb-dist* 2.0)) 0 0.0))
	   (send next-cds :translate (float-vector (/ *ideal-fb-dist* 2.0) 0 0.0))
	   (let ((res (send self :calc-gripper-trajectory next-cds (* 1.5 *ideal-fb-dist*))))
	     (if res
		 (progn
		   (setq avs-new res)
		   (setq tms-new (make-list (length avs-new) :initial-element t-gap))
		   ;;adjust t-check
		   ;;dobule transition time
		   ;; (setq t-check (ros::time+ t-check (ros::time (/ (* 1 (elt tms-new 0)) 1000.0))))
		   ;; (setf (elt tms-new 0) (* 2 (elt tms-new 0)))

		   (let* ((c (send *pr2* *use-arm* :end-coords :worldpos))
			  (d (norm (v- (float-vector (elt c 0) (elt c 1) 0.0) (send *pr2* :worldpos)))))
		     (when (< d 400)
		       (print "gripper is too close to body")
		       (if (= loop-cnt 0)
			   (return-from :move-object "move-base-farther")
			 (return-from :move-object "move-base")))))
	       (progn
		 (print "failed to solve ik")
		 (if (= loop-cnt 0)
		     (return-from :move-object "move-base-closer")
		   (return-from :move-object "move-base"))))))
	 
	 (send obj-cds-pre :move-to (send obj-cds :copy-worldcoords) :world)
	 (send *obj-cds-pre* :move-to (send obj-cds-pre :copy-worldcoords) :world)
	 (let ((cube-now (make-cube 10 10 10)))
	       (send cube-now :move-to (send *cube-obj-pre* :copy-worldcoords))
	       (send cube-now :set-color :red)
	       (send *irtviewer* :objects cube-now))
	 (send *cube-obj-pre* :move-to *obj-cds-pre* :world)
	 (send *irtviewer* :draw-objects :flush t)


	 (ros::ros-warn "t-check - after:~A~%" (ros::time- t-check (ros::time-now)))
	 ;;connect two angle-vector-sequences
	 (unless (= (length avs-pre) 0)
	   (let* ((t-now (ros::time-now))
		  (st1 st-time-pre)
		  (st2 (ros::time- t-check (ros::time (/ *fb-period* 1000.0 2.0))))
		  (tms1 tms-pre)
		  (tms2 tms-new)
		  et1
		  (n 0)
		  (i 0)
		  (t-acc st1)
		  (avs1 avs-pre)
		  (avs2 avs-new))
	     (dolist (tm tms1)
	       (setq t-acc (ros::time+ t-acc (ros::time (/ tm 1000.0))))
	       (when (ros::time> t-acc t-now)
		 (return))
	       (incf n))
	     (print n)
	     (print (length tms1))
	     (if (= n 0)
		 (progn
		   (setq tms1 (subseq tms1 n (1+ n)))
		   (setq avs1 (subseq avs1 n (1+ n))))
	       (progn
		 (setq st1 (ros::time- t-acc (ros::time (/ (+ (elt tms1 (1- n)) (elt tms1 n)) 1000.0))))
		 (if (>= (- (length tms1) n) 2)
		     (progn
		       (setq tms1 (subseq tms1 (- n 1) (+ n 2)))
		       (setq avs1 (subseq avs1 (- n 1) (+ n 2))))
		   (progn
		     (setq tms1 (subseq tms1 (- n 1)))
		     (setq avs1 (subseq avs1 (- n 1)))))))
	     (setq et1 st1)
	     (dolist (tm tms1)
	       (setq et1 (ros::time+ et1 (ros::time (/ tm 1000.0)))))
	     (setq t-acc st2)
	     (dolist (tm tms2)
	       (setq t-acc (ros::time+ t-acc (ros::time (/ tm 1000.0))))
	       (print t-acc)
	       (print et1)
	       (print "check")
	       (when (ros::time> t-acc et1)
		 (return))
	       (incf i))
	     (format t "i=~A~%" i)
	     (setq st-time-new st1)
	     (setq avs-new (append avs1 (subseq avs2 i)))
	     (print (ros::time- t-acc et1))
	     (print tms1)
	     (print (max (* 1 t-gap) (* 1e-6 (send (ros::time- t-acc et1) :to-nsec))))
	     (print (subseq tms2 (1+ i)))

	     ;; (setq tms-new (append tms1 (list (+ t-shift (* 1e-6 (send (ros::time- t-acc et1) :to-nsec)))) (subseq tms2 (1+ i))))))
	     (let ((t-shift (max (* 1 t-gap) (* 1e-6 (send (ros::time- t-acc et1) :to-nsec)))))
	       (setq tms-new (append tms1 (list t-shift) (subseq tms2 (1+ i)))))))
	       ;; (if (= t-shift (* 1 t-gap))
	       ;; 	   (setq t-check (ros::time+ t-check (ros::time (/ t-gap 1000.0))))))))
	 (setq t-now (ros::time-now))
	 (format t "after:~A~%" t-now)
	   ;; (unless (= (length avs-pre) 0)
	   ;;   (let ((t-acc 0)
	 ;; 	   (len (max (/ (length avs-new) 2) 3)))
	   ;;     (format t "length avs-new:~A~%" (length avs-new))
	   ;;     (dolist (tm (subseq tms-pre 0 (- (length tms-pre) len)))
	   ;; 	 (setq t-acc (+ t-acc tm)))
	   ;;     (setq st-time-new (ros::time+ st-time-pre (ros::time (/ t-acc 1000.0))))
	   ;;     (setq avs-new (append (subseq avs-pre (- (length avs-pre) len)) avs-new))
	   ;;     (setq tms-new (append (subseq tms-pre (- (length avs-pre) len)) tms-new))))
	 ;; (print avs-new)
	 ;; (print tms-new)
	;;(print st-time-new)
	 ;; (print (length avs-new))
	 ;; (print (length tms-new))
	 
	 
 	   (send *ri* :angle-vector-sequence avs-new tms-new  :default-controller st-time-new)
	   (setq st-time-pre st-time-new)
	   (setq avs-pre avs-new)
	   (setq tms-pre tms-new)
	   (send *obj-trans* :move-to (send obj-trans :copy-worldcoords) :world)
	   
	   (ros::ros-warn "t-check - after:~A~%" (ros::time- t-check (ros::time-now)))
	   ;; (let ((dt (ros::time- (ros::time-now) t-check)))
	   ;;   (if (ros::time> dt (ros::time 0))
	   ;; 	 (progn
	   ;; 	   (setq t-check (ros::time+ t-check dt)))))
	   (incf loop-cnt)
	   )
       (send *ri* :spin-once)
       (ros::sleep)
       )))


  (:publish-manipulate-arrow
   ()
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (let ((box (instance jsk_pcl_ros::BoundingBox :init))
	 (header (instance std_msgs::Header :init))
	 (cds (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
     
     (send header :frame_id "base_footprint")
     (send header :stamp (ros::time-now))
     (send box :header header)
     (send cds :transform (send (send *pr2* :copy-worldcoords) :inverse-transformation) :world)
     (send box :pose (ros::coords->tf-pose cds))
     (send box :dimensions :x 0.05)
     (send box :dimensions :y 0.05)
     (send box :dimensions :z 0.05)
     (ros::publish "/ground_object_segmentation/bounding_box_marker/draw_manipulate_arrow" box)))
     

  (:calc-gripper-trajectory
   (tgt-cds d &optional (n *ik-division*)) ;start-coords distance[mm]
   (let* ((obj-cds (make-cascoords :coords (send tgt-cds :copy-worldcoords)))
	  (c (send (send obj-cds :copy-worldcoords) :transform *obj-gripper-trans*))
	  (gripper-cds (make-cascoords :coords c))
	  av-seq)
     ;;(send *irtviewer* :objects obj-cds)
     (send obj-cds :assoc gripper-cds)
     (send obj-cds :translate (float-vector (/ (* 1.0 d) n) 0 0))
     (dotimes (i n)
       ;;(send *irtviewer* :draw-objects :flush t)
       (unless (send *pr2* *use-arm* :inverse-kinematics gripper-cds :look-at-target t)
	 (return-from :calc-gripper-trajectory nil))
       (push (send *pr2* :angle-vector) av-seq)
       (send obj-cds :translate (float-vector (/ (* 1.0 d) n) 0 0)))
     (reverse av-seq)))

  

  (:update-base-trans 
   ()
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (let* ((base-cds (send *pr2* :copy-worldcoords))
	  (gripper-pos-cds (send (make-coords :pos (send *pr2* *use-arm* :end-coords :worldpos) :rot (send base-cds :worldrot)) :copy-worldcoords)))
     (setq *base-trans* (send gripper-pos-cds :transformation base-cds))))


  (:hold-arm-move-base 
   (tgt-cds)
   (let* ((base-recent (send *pr2* :copy-worldcoords))
	  (base-tgt (send tgt-cds :copy-worldcoords))
	  (diff-cds (send base-recent :transformation base-tgt))
	  (dif-v (send diff-cds :pos))
	  (gripper-cds (send *pr2* *use-arm* :end-coords :copy-worldcoords))
	  (move-time 3000.0)
	  )
     
     (send *pr2* :move-to base-tgt :world)
     (send *pr2* *use-arm* :inverse-kinematics gripper-cds :look-at-target t)
     (let ((sttm (ros::time+ (ros::time-now) (ros::time 0.5))))
       (send *ri* :angle-vector (send *pr2* :angle-vector)
	     move-time :default-controller sttm)
       (send *ri* :move-trajectory
	     (/ (elt dif-v 0) move-time) (/ (elt dif-v 1) move-time) (/ (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1)) (/ move-time 1000.0))
	     move-time :start-time sttm :send-action t)
       )))
  
  (:calc-trans-from-pos-error 
   (pre-cds cds)
   (let* ((diff-cds (make-coords))
	  (output-trans (make-coords))
	  (dif-v (send (send pre-cds :transformation cds) :pos))
	  (v-proj (float-vector (elt dif-v 0) (elt dif-v 1) 0.0))
	  (ang-z (vector-angle #f(1 0 0) v-proj #f(0 0 1)))
	  ang-y
	  ang-sum
	  ang-pid-out
 	  ;;(dist-gain (/ (norm dif-v) *ideal-fb-dist*))
	  (dist-gain (/ (norm dif-v) 100.0))
	  (k-gain (* 0.45 *k-gain-f*))
	  (i-gain (* (* (/ *fb-period* 1000.0)) *k-gain-f*))
	  ;;(i-gain (* (/ (+ *t-long* (* *fb-inr* *t-short*) *t-gap*) 1000.0) *k-gain-f*))
	  p-factor
	  i-factor)

     ;;(setq dist-from-pre (norm (v- (send obj-cds-pre :worldpos) (send obj-cds :worldpos))))
     (send diff-cds :rotate ang-z :z)
     (setq dif-v (send diff-cds :inverse-transform-vector dif-v))
     (setq ang-y (vector-angle #f(1 0 0) dif-v #f(0 1 0)))
     (send diff-cds :rotate ang-y :y)
     (push dif-v dif-v-seq)
     (push diff-cds dif-cds-seq)
     (push (float-vector ang-z ang-y) *ang-seq*)
     

     (format t "dist-gain:=~A~%" dist-gain)
     ;;(setq dist-gain (min 1.0 dist-gain))
     (setq dist-gain (min (/ *ideal-fb-dist* 100.0) dist-gain))
     (format t "dist-gain:=~A~%" dist-gain)
     (setq p-factor (car *ang-seq*))
     (let ((sum #f(0 0)))
       (dolist (ang *ang-seq*)
	 (setq sum (v+ sum ang)))
       (setq i-factor (scale (/ *fb-period* 1000.0) sum))

     (setq ang-pid-out (scale (* dist-gain k-gain) (v+ p-factor (scale (/ 1 i-gain) i-factor))))
     
     (send output-trans :rotate (elt ang-pid-out 0) :z)
     (send output-trans :rotate (elt ang-pid-out 1) :y)
     ;; (send output-trans :rotate (* dist-gain k-gain (+ (elt p-factor 0) (/ (elt i-factor 0) i-gain))) :z)
     ;; (send output-trans :rotate (* dist-gain k-gain (+ (elt p-factor 1) (/ (elt i-factor 1) i-gain))) :y)
     (format t "ang-recent:~A~%" (mapcar #'rad2deg (coerce (car *ang-seq*) cons)))
     (format t "ang-sum   :~A~%" (mapcar #'rad2deg (coerce sum cons)))
     (format t "ang-pid :~A~%" (mapcar #'rad2deg (coerce ang-pid-out cons)))
     output-trans)))


  (:test ()
	 (do-until-key
	  (send *irtviewer* :objects (list *pr2* *cube-obj-pre*))
	  (send *ri* :angle-vector (send *pr2* :reset-pose))
	  (send *ri* :wait-interpolation)
	  (send self :init-obj-cds)
	  ;;(send *manipulate-cds* :rotate (deg2rad -10) :y)
	  (send self :move-object)))
)

