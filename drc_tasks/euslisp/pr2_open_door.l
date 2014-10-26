(defun init ()
  (send *pr2* :reset-pose)
  (setq tgt (make-cube 20 20 20))
  (send tgt :set-color :red)
  (send tgt :locate (send *pr2* :rarm :end-coords :worldpos))
  (send tgt :translate #f(200 200 100))
  (send tgt :rotate (deg2rad 90) :x)
;  (setq *irtviewer* (make-irtviewer))
  (send *pr2* :larm :inverse-kinematics (send tgt :copy-worldcoords) :use-torso t)
)

(defun rotate-knob (r tgt-deg)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)) 
  (let ((d 0) 
	(c_handle (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
	(c_hand (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
	(av-seq nil))
    (send c_handle :assoc c_hand)
    (send *irtviewer* :objects c_handle c_hand)
    (while (< d tgt-deg)
      (setq d (+ d 1))
      (send c_handle :rotate (deg2rad -1) :x :world)
      (send c_handle :translate (float-vector 0.0 0.0 (- (* r (sin (deg2rad 1))))))
      (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords) :rotation-axis t)
      (setq av-seq (append av-seq (list (send *pr2* :angle-vector))))
      (send *irtviewer* :draw-objects :flush t))
    (send *ri* :angle-vector-sequence av-seq 100)
    ))

(defun pull-door (r tgt-deg &optional (pull t) (axis-up nil))
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((d 0)
	(c_hand (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
	(c_door (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
	(c_base (make-cascoords))
	(av-seq (list (send *pr2* :angle-vector)))
	ret
	(move-dist 150.0)
	(move-time 1500))
    (send *irtviewer* :objects c_hand c_door)
    (send c_door :assoc c_hand)
    (while (< d tgt-deg)
      (setq d (+ d 1))
      (send c_door :rotate (deg2rad -1) :z)
      (send c_door :translate (float-vector (- (* r (sin (deg2rad 1)))) 0.0 0.0))
      (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords))
      ;; move base if ik unsolved or arm too close
      (unless (and (> (norm (v- (send *pr2* :larm :end-coords :worldpos) (send *pr2* :worldpos))) 1100))
	(send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords))
      	(send *ri* :angle-vector-sequence av-seq 100)
      	(send *ri* :wait-interpolation)
      	(setq av-seq nil)
      	(send c_base :translate (float-vector (- move-dist) move-dist 0))
      	(send *pr2* :move-to c_base :world)
      	(setq ret (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords)))
      	(if ret
      	    (progn
      	      (let ((sttm (ros::time+ (ros::time-now) (ros::time 0.5))))
      		(send *ri* :angle-vector (send *pr2* :angle-vector)
      		      move-time :default-controller sttm)
      		(send *ri* :move-trajectory
      		      (- (/ move-dist move-time)) (/ move-dist move-time) 0.0
      		      move-time :start-time sttm :send-action t)))
      	  (progn
      	    (print "Open door failed")
      	    (return-from pull-door nil))))
      (setq av-seq (append av-seq (list (send *pr2* :angle-vector))))
      (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector-sequence av-seq 100)
    (send *ri* :wait-interpolation)

    ;;if real-world and model angle-vector is different
    (when (> (norm (v- (send *pr2* :angle-vector) (send *ri* :state :potentio-vector))) 25)
      (send c_base :translate (float-vector 0.0 300 0))
      (send *pr2* :move-to c_base :world)
      (setq ret (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords)))
      (let ((sttm (ros::time+ (ros::time-now) (ros::time 0.5))))
	(send *ri* :angle-vector (send *pr2* :angle-vector)
	      move-time :default-controller sttm)
	(send *ri* :move-trajectory
	      (- (/ move-dist move-time)) (/ move-dist move-time) 0.0
	      move-time :start-time sttm :send-action t)))
    ))


(defun open-b2-door ()
  (send *ri* :start-grasp :larm :gain 0.1)
  (rotate-knob 30 45)
  (send *ri* :wait-interpolation)
  (pull-door 730 100)
  
  ;;check if door is fully open
  (let (pre-robot-av diff)
    (dotimes (i 2)
      (setq pre-robot-av (send *ri* :state :potentio-vector))
      (send *pr2* :angle-vector pre-robot-av)
      (send *pr2* :larm :inverse-kinematics 
	    (send (send *pr2* :larm :end-coords :copy-worldcoords)
		  :translate #f(0.0 50 0.0) :world))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
      (unix:usleep (* 500 1000))
      (setq diff (norm (v- pre-robot-av (send *ri* :state :potentio-vector))))
      (if (< diff 15) (return 1)))

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :stop-grasp :larm)
  (send *pr2* :larm :inverse-kinematics 
	(send (send *pr2* :larm :end-coords :copy-worldcoords)
	      :translate #f(0.0 100 0.0) :world)
	:rotation-axis nil)
  (send *pr2* :go-pos-unsafe -0.2 0.0)
  (pr2-reset-pose)
  (pr2-tuckarm-pose)
  (send *ri* :go-pos-unsafe 0.0 (/ (elt (send *pr2* :worldpos) 1) -1000.0))
  )
    
     

   
      
      
      
      
      





      
    
    
      
    
  
      
