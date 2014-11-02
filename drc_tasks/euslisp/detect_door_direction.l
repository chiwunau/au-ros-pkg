(defun test-door-direction (r) ;;by pushing or pulling 10 cm and see the displacement of av
  (let ((av-org (send *ri* :state :potentio-vector))
	cds_world cds_tgt av-seq d res move_dist dis)
    (dotimes (i 2)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (setq cds_world (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
      (setq cds_tgt (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
      (send cds_world :assoc cds_tgt)
      (setq av-seq (list (send *pr2* :angle-vector)))
      (setq move_dist (* r (sin (deg2rad 10))))
      (dotimes (d 10)
	(send cds_world :translate (float-vector 
				    (* (if (= i 0) -1 1) r (sin (deg2rad 1)))  0.0 0.0))
	(send *pr2* :larm :inverse-kinematics (send cds_tgt :copy-worldcoords) :rotation-axis t)
	(setq av-seq (append av-seq (list (send *pr2* :angle-vector)))))
      (print av-seq)
      (send *ri* :angle-vector-sequence av-seq 100)
      (send *ri* :wait-interpolation)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (setq res (send *pr2* :larm :end-coords :copy-worldcoords))
      (setq dis (v- (send res :worldpos) (send cds_tgt :worldpos)))
      
      (cond 
       ((and (= i 0) (< (abs (elt dis 0)) (/ move_dist 1.5)) (> (elt dis 1) 0))
	(ros::ros-warn "pull: t axis-up: nil")
	(send *ri* :angle-vector av-org 1000)
	(return-from test-door-direction (list t nil)))
       ((and (= i 0) (< (abs (elt dis 0)) (/ move_dist 1.5)) (< (elt dis 1) 0))
	(send *ri* :angle-vector av-org 1000)
	(ros::ros-warn "pull: t axis-up: t")
	(return-from test-door-direction (list t t)))
       ((and (= i 1) (< (abs (elt dis 0)) (/ move_dist 1.5)) (> (elt dis 1) 0))
	(send *ri* :angle-vector av-org 1000)
	(ros::ros-warn "pull: nil axis-up: t")
	(return-from test-door-direction (list nil t)))
       ((and (= i 1) (< (abs (elt dis 0)) (/ move_dist 1.5)) (< (elt dis 1) 0))
	(send *ri* :angle-vector av-org 1000)
	(ros::ros-warn "pull: nil axis-up: nil")
	(return-from test-door-direction (list nil nil)))
       (t 
	(send *ri* :angle-vector av-org 1000)
	(send *ri* :wait-interpolation))
       ))
  (ros::ros-warn "failed to detect direction of door")
  ))

(defun test-knob-direction (&optional (use-arm *use-arm*))
  (let (cds_gripper
	cds_knob
	(pre-av (send *ri* :state :potentio-vector))
	pos-diff0
	pos-diff1)

    (dotimes (i 2)
      (send *pr2* :angle-vector pre-av)      
      (if (= i 0)
	  (progn
	    (setq tgt-pos (rotate-knob 10 t))
	    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	    (setq pos-diff0 (norm (v- (send *pr2* use-arm :end-coords :worldpos) (send tgt-pos :worldpos)))))
	(progn
	  (setq tgt-pos (rotate-knob 10 nil))
	  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	  (setq pos-diff1 (norm (v- (send *pr2* use-arm :end-coords :worldpos) (send tgt-pos :worldpos))))))
	
      (send *ri* :angle-vector pre-av 500)
      (send *ri* :wait-interpolation))
    (if (< pos-diff0 pos-diff1)
	(progn
	  (ros::ros-warn "knob-axis: plus")
	  (setq *knob-axis-plus* t)
	  (return-from test-knob-direction t))
      (progn
	(ros::ros-warn "knob-axis: minus")
	(setq *knob-axis-plus* nil)
	(return-from test-knob-direction nil)))))
	
      

	 

      
