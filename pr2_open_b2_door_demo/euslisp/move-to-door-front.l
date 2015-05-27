(require "package://jsk_demo_common/euslisp/pr2-move.l")
(require "package://jsk_perception/euslisp/detection_interface.l")
(require "package://pr2eus/speak.l")

(defparameter *detection-topic* "/door_detect/ObjectDetection")

(defun look-around-for-checkerboard ()
  (send (check-detection :type "door_checkerboard"
                                   :speak-name "door"
                                   :timeout 5
                                   :speak nil) :copy-worldcoords))

(defun move-to-door-grasp-knob ()
  (let (ckb-cds
        base-target-cds
        knob-cds)
    ;; (speak-jp "ドアに移動します")
    (send *pr2* :head :neck-p :joint-angle 0)
    (send *pr2* :head :neck-y :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (pr2-tuckarm-pose :larm)
    ;; (move-to-spot "/eng2/7f/room73B2-door-front"
    ;;               :clear-costmap t
    ;;               :tilt-enable t)

    ;;adjust base location
    (setq ckb-cds (look-around-for-checkerboard))
    (objects ckb-cds)
    (setq base-target-cds (send ckb-cds :copy-worldcoords))
    (send base-target-cds :rotate (deg2rad 90) :x)
    (send base-target-cds :rotate (deg2rad 90) :z)
    (setf (elt (send base-target-cds :worldpos) 2) 0)
    (send base-target-cds :translate #f(-800 0 0))
    (send base-target-cds :translate #f(0 100 0))
    (objects base-target-cds)
    (let ((diff-cds (send (send *pr2* :copy-worldcoords) :transformation base-target-cds)))
      (send *ri* :go-pos-unsafe
            (/ (elt (send diff-cds :pos) 0) 1000)
            (/ (elt (send diff-cds :pos) 1) 1000)
            (rad2deg (vector-angle #f(1 0 0) (matrix-column (send diff-cds :rot) 0) #f(0 0 1)))))
    ;;setq knob coordinates
    (send ckb-cds :move-to (look-around-for-checkerboard) :world)
    (send *irtviewer* :draw-objects :flush t)
    (setq knob-cds (send ckb-cds :copy-worldcoords))
    (send knob-cds :rotate (deg2rad 90) :x)
    (send knob-cds :rotate (deg2rad 90) :z)
    (send knob-cds :translate #f(-55 -25 -400))
    (send knob-cds :rotate (deg2rad -90) :x)
    (send *pr2* :larm :inverse-kinematics (send (send knob-cds :copy-worldcoords) :translate #f(-80 0 0)))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :larm)
    (unix::usleep  (* 1000 1000))
    (send *pr2* :larm :inverse-kinematics (send (send knob-cds :copy-worldcoords) :translate #f(35 0 0)) :use-torso 0.0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :larm)


    

    
    
    ;; (send base-target-cds :transate)
)


         
