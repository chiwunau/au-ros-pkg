;; This buffer is for notes you don't want to save, and for Lisp evaluation.
;; If you want to create a file, visit that file with C-x C-f,
;; then enter the text in that file's own buffer.
(load "make-door.l")
(load "real-endpos-feedback.l")
(load "detect_door_direction.l")
(defvar *door* nil)
(defvar *door-radius* 730)
(defvar *knob-radius* 40)
(defvar *door-max-angle* nil)
(defvar *knob-max-angle* nil)
(defvar *door-pull* t)
(defvar *door-axis-up* nil)
(defvar *knob-axis-plus* nil)
(defvar *door-trajectory* nil)
(defvar  *use-arm* :larm)

(defvar *cds-knob-init* nil)
(defvar *cds-gripper-init* nil)
(if (not (boundp '*irtviewer*))
    (setq *irtviewer* (make-irtviewer)))

(defun init-door-model ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 200)
  (setq *cds-gripper-init* (send *pr2* *use-arm* :end-coords :copy-worldcoords))
  (setq *cds-knob-init* (send *pr2* :larm :end-coords :copy-worldcoords))
  (setq *door* (instance door :init *door-radius* (elt (send *cds-knob-init* :worldpos) 2)))
  (send (car (send *door* :links)) :locate (scale -1 (send *door* :end-coords :worldpos)))
  (send (car (send *door* :links)) :locate (send *pr2* :larm :end-coords :worldpos))
  (draw-trajectory (calc-door-trajectory *door-radius* *door-pull* *door-axis-up*))
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

(defun rotate-knob (tgt-deg axis-p &optional (play-back nil) (r *knob-radius*) (use-arm *use-arm*))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((d 0)
	(cds-gripper (make-cascoords :coords(send *pr2* :larm :end-coords :copy-worldcoords)))
	cds-knob 
	(av-seq nil))
    (print r)

    ;;make x-axis of cds-knob become axis of knob
    (setq cds-knob (make-cascoords :coords (send cds-gripper :copy-worldcoords)))
    ;;if z-axis of gripper point left
    (if (> (elt (matrix-row (transpose (send cds-gripper :rot)) 2) 1) 0) 
	(progn
	  (send cds-knob :rotate (deg2rad 90) :y)
	  (send cds-knob :rotate (deg2rad 90) :z))
      (send cds-knob :rotate (deg2rad -90) :x))
    (send cds-knob :translate (float-vector 0.0 (if axis-p r (* -1 r)) 0.0))
    ;;finished seting coords

    (send cds-knob :assoc cds-gripper)
      ;; (send *irtviewer* :objects c_handle c_hand)
    (while (< d tgt-deg)
      (incf d)
      (send cds-knob :rotate (deg2rad (if axis-p 1 -1)) :x :world)
      (send cds-knob :translate (float-vector 0.0 0.0 (* -1 r (sin (deg2rad 1)))))
      (send *pr2* use-arm :inverse-kinematics (send cds-gripper :copy-worldcoords) :rotation-axis t)
      (setq av-seq (append av-seq (list (send *pr2* :angle-vector))))
      (send *irtviewer* :draw-objects :flush t))
    (send *ri* :angle-vector-sequence av-seq 100)
    (send *ri* :wait-interpolation)
    (if play-back
	(progn
	  (reverse av-seq)
	  (send *ri* :angle-vector-sequence av-seq 100)
	  (send *ri* :wait-interpolation)
	  ))
    (send *pr2* use-arm :end-coords :copy-worldcoords)
    ))


