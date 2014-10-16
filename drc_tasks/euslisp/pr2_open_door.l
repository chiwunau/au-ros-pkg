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
	(c (make-coords :pos (send *pr2* :larm :end-coords :worldpos)))
	(av-seq nil))
    (send *irtviewer* :objects c)
    (while (< d tgt-deg)
      (setq d (+ d 1))
      (send c :rotate (deg2rad -1) :x :world)
      (send c :translate (float-vector 0.0 (- (* r (sin (deg2rad 1)))) 0.0))
      (send *pr2* :larm :inverse-kinematics c)
      (setq av-seq (append av-seq (list (send *pr2* :angle-vector))))
      (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector-sequence av-seq 100)
    ))

(defun pull-door (r tgt-deg)
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((d 0)
	(c_hand (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
	(c_door (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
	(c_base (make-cascoords))
	(av-seq (list (send *pr2* :angle-vector))))
    (send *irtviewer* :objects c_hand c_door)
    (send c_door :assoc c_hand)
    (while (< d tgt-deg)
      (setq d (+ d 1))
      (send c_door :rotate (deg2rad -1) :z)
      (send c_door :translate (float-vector (- (* r (sin (deg2rad 1)))) 0.0 0.0))
      (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords))
      ;;move base if ik unsolved or arm too close
      (unless (and (> (norm (v- (send *pr2* :larm :end-coords :worldpos) (send *pr2* :worldpos))) 1100)
		   (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords)))
	(send *ri* :angle-vector-sequence av-seq 200)
	(send *ri* :wait-interpolation)
	(setq av-seq nil)
	(send c_base :translate #f(-200 200 0))
	(send *pr2* :move-to c_base :world)
	(if (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords))
	    (progn
	      (send *ri* :go-pos-unsafe
		    (/ -200.0 1000)
		    (/ 200.0 1000)
		    (/ 0.0 1000)
		    :gain 0.5)
					;(unix:sleep 2)
	      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
	      (send *ri* :wait-interpolation)
	      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	      (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords))
	      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000))
	  (progn
	    (print "Open door failed")
	    (return-from pull-door nil))))
      (setq av-seq (append av-seq (list (send *pr2* :angle-vector))))
      (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector-sequence av-seq 200)
    ))

;; (defun move-back-open-door (r tgt-deg)
;;   (send *pr2* :move-to (make-coords) :world)
;;   ;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;   (let ((d 0)
;; 	(c_hand (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
;; 	(c_door (make-cascoords :pos (send *pr2* :larm :end-coords :worldpos)))
;; 	(c_base (make-cascoords))
;; 	(send *irtviewer* :objects c_hand c_door c_base)
;; 	(av-seq (list (send *pr2* :angle-vector))))
;;     (send c_door :assoc c_hand)
;;     (while (< d 10)
;;       (setq d (+ d 1))
;;       (send c_door :rotate (deg2rad -1) :z)
;;       (send c_door :translate (float-vector (- (* r (sin (deg2rad 1)))) 0.0 0.0))
;;       (send *pr2* :larm :inverse-kinematics (send c_hand :copy-worldcoords))
;;       (setq av-seq (append av-seq (list (send *pr2* :angle-vector))))
;;       (send *irtviewer* :draw-objects))
;;     ;;(send *ri* :angle-vector-sequence av-seq 100)
;;     ;;(send *ri* :wait-interpolation)
;;     ;;start go back
;;     (read)
;;     (send c_door :assoc c_base)
;;     (while (< d tgt-deg)
;;       (setq d (+ d 1))
;;       (send c_door :rotate (deg2rad -1) :z)
;;       (send c_door :translate (float-vector (- (* r (sin (deg2rad 1)))) 0.0 0.0))
;;       (send *pr2* :move-to (send c_base :copy-worldcoords)  :world)
;;       (send *irtviewer* :draw-objects))
;;     ))

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
	  
    
     

   
      
      
      
      
      





      
    
    
      
    
  
      
