;; This buffer is for notes you don't want to save, and for Lisp evaluation.
;; If you want to create a file, visit that file with C-x C-f,
;; then enter the text in that file's own buffer.
(load "make-door.l")
(setq *door* nil)
(setq *door-radius* 730)
(setq *knob-radius* 40)
(setq *door-max-angle* nil)
(setq *knob-max-angle* nil)
(setq *door-pull* t)
(setq *door-axis-up* nil)
(setq *door-trajectory* nil)
(setq *use-arm* :larm)

(setq *cds-knob-init* nil)
(setq *cds-gripper-init* nil)
(if (not (boundp '*irtviewer*))
    (setq *irtviewer* (make-irtviewer)))

(defun init-door-model ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 200)
  (setq *cds-gripper-init* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
  (setq *cds-knob-init* (send *pr2* :larm :end-coords :copy-worldcoords))
  (setq *door* (instance door :init *door-radius* (elt (send *cds-knob* :worldpos) 2)))
  (send (car (send *door* :links)) :locate (scale -1 (send *door* :end-coords :worldpos)))
  (send (car (send *door* :links)) :locate (send *pr2* :larm :end-coords :worldpos))
  (setq *door-trajectory* (calc-door-trajectory *door-radius* *door-pull* *door-axis-up*))
  (send *irtviewer* :objects nil)
  (draw-trajectory *door-trajectory*)
  (send *irtviewer* :objects *pr2* *door*))

(defun calc-and-draw-real-state ()
  (let (res ang)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq res (obtain-real-angle-from-trajectory *door-trajectory*))
    (setq ang (car res))
    (send (car (send *door* :joint-list)) :joint-angle (* -1 ang))
    (send *irtviewer* :draw-objects :flush t)
    res
    ))

(defun rotate-knob ()
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

(defun pull-door-one-action 
  (let 
  