(defclass door
  :super cascaded-link
  :slots (end-coords l1 l2 l3 j1 j2))

(defmethod door
  (:init (r h)
	 (let (b)
	   (print r)
	   (send-super :init)
	   (setq b (make-cube 20 100 20))
	   (send b :locate #f(0 50 0))
	   (send b :set-color :red)
	   (setq l3 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-knob))
	   (setq end-coords (make-cascoords :pos #f(0 50 0)))
	   (send l3 :assoc end-coords)
	   (send l3 :locate (float-vector -35 (* -1 r) h))
	   
	   (setq b (make-cube 50 (+ r 20) (* 2 h)))
	   (send b :locate (float-vector 0 (/ (+ r 20) -2.0) h))
	   (send b :set-color :green)
	   (setq l2 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-plate))
	   (send l2 :assoc l3)
	   (send l2 :locate #f(0 -25 0))
	   
	   (setq b (make-cube 50 50 (* 2 h)))
	   (send b :locate (float-vector 0 0 h))
	   (send b :set-color :grey)
	   (setq l1 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-hinge))
	   (send l1 :assoc l2)
	   
	   (setq j1 (instance rotational-joint :init :name 'j1
			      :parent-link l1 :child-link l2 :axis :z :min -100 :max 100))
	   (setq j2 (instance rotational-joint :init :name 'j1
			      :parent-link l2 :child-link l3 :axis :x :min -45 :max 0))
	   
	   (setq links (list l1 l2 l3))
	   (setq joint-list (list j1 j2))
	   (send self :init-ending)
	   self))
  (:end-coords (&rest args) (forward-message-to end-coords args))
  )


