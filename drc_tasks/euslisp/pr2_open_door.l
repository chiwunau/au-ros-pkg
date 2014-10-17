(defun init ()
  (send *pr2* :reset-pose)
  (setq tgt (make-cube 20 20 20))
  (send tgt :set-color :red)
  (send tgt :locate (send *pr2* :rarm :end-coords :worldpos))
  (send tgt :translate #f(200 100 0))
  (send tgt :rotate (deg2rad 90) :x)
;  (setq *irtviewer* (make-irtviewer))
  (send *pr2* :larm :inverse-kinematics (send tgt :copy-worldcoords))
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

(defun pull-door (r tgt-deg)
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((d 0)
	(c_hand (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
	(c_door (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
	(c_base (make-cascoords))
	(av-seq (list (send *pr2* :angle-vector)))
	ret
	(move-dist 200.0))
    (send *irtviewer* :objects c_hand c_door)
    (send c_door :assoc c_hand)
    (while (< d tgt-deg)
      (setq d (+ d 1))
      (send c_door :rotate (deg2rad -1) :z)
      (send c_door :translate (float-vector (- (* r (sin (deg2rad 1)))) 0.0 0.0))
      (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords))
      move base if ik unsolved or arm too close
      (unless (and (> (norm (v- (send *pr2* :larm :end-coords :worldpos) (send *pr2* :worldpos))) 1100)
      		   (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords)))
      	(send *ri* :angle-vector-sequence av-seq 200)
      	(send *ri* :wait-interpolation)
      	(setq av-seq nil)
      	(send c_base :translate (float-vector (- move-dist) move-dist 0))
      	(send *pr2* :move-to c_base :world)
      	(setq ret (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords)))
      	(if ret
      	    (progn
      	      (let ((sttm (ros::time+ (ros::time-now) (ros::time 1.0))))
      		(send *ri* :angle-vector (send *pr2* :angle-vector)
      		      2000 :default-controller sttm)
      		(send *ri* :move-trajectory
      		      (- (/ move-dist 1000)) (/ move-dist 1000) 0.0
      		      2000 :start-time sttm :send-action t)))
      	  (progn
      	    (print "Open door failed")
      	    (return-from pull-door nil))))
      (setq av-seq (append av-seq (list (send *pr2* :angle-vector))))
      (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector-sequence av-seq 200)
    ))


(defun move-base-open-door (r tgt-deg)
  (send *pr2* :move-to (make-coords) :world)
  (let ((d 0)
	(c_door (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
	(c_base (make-cascoords))
	(pre_c_base (make-cascoords)))
    (send *irtviewer* :objects c_door c_base)
    (send c_door :assoc c_base)
    (while (< d tgt-deg)
      (setq d (+ d 1))
      (send c_door :rotate (deg2rad -1) :z)
      (send c_door :translate (float-vector (- (* r (sin (deg2rad 1)))) 0.0 0.0))
      (send *pr2* :move-to (send c_base :copy-worldcoords) :world)
      (send *irtviewer* :draw-objects)
      (if (= (mod d 30) 0)
	  (progn 
	    (setq diff (scale 0.001 (v- (send c_base :worldpos) (send pre_c_base :worldpos))))
	    (send *ri* :go-pos-unsafe 
		  (elt diff 0)
		  (elt diff 1)
		  -30)
	    ;(send *ri* :wait-interpolation)
	    (send pre_c_base :newcoords (send c_base :copy-worldcoords)))))
    (setq diff (scale 0.001 (v- (send c_base :worldpos) (send pre_c_base :worldpos))))
    (send *ri* :go-pos-unsafe 
	  (elt diff 0)
	  (elt diff 1)
	  (* -1 (mod d 30)))))
	  
    
     

   
      
      
      
      
      





      
    
    
      
    
  
      
