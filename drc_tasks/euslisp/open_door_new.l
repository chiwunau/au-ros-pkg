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

(setq *cds-knob* nil)

(if (not (boundp '*irtviewer*))
    (setq *irtviewer* (make-irtviewer)))

(defun init-door-model ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *cds-knob* (send *pr2* :larm :end-coords :copy-worldcoords))
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

  