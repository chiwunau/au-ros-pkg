(ros::load-ros-manifest "drc_tasks")
(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus "subscribe_plane_to_eus")

(setq *vertices-list* nil)
(setq *plane-normal-list* nil)
(setq *polygons-list* nil)
(setq *cloud-stamp* nil)
(setq *cloud-centroid-list* nil)

(setq *plgs-cb-flag* nil)
(setq *cofs-cb-flag* nil)


(defun polygons-array-cb (msg)
  (setq *plgs-cb-flag* t)
  (setq *cloud-stamp* (send msg :header :stamp))
  (let ((plg-list (send msg :polygons)))
    (setq *vertices-list* (mapcar #'(lambda (plg)
				       (let ((pts (send (send plg :polygon) :points))
					     (pts-list))
					 (dolist (pt pts)
					   (push 
					    (send (send *pr2* :copy-worldcoords) :transform-vector 
						  (scale 1000 (float-vector (send pt :x) (send pt :y) (send pt :z))))
					    pts-list))
					 (reverse pts-list)))
				   plg-list))))

(defun coefficients-array-cb (msg)
  (setq *cofs-cb-flag* t)
  (let ((cof-list (send msg :coefficients))
	normal-list)
    (dolist (cof cof-list)
      (let ((val (send cof :values)))
	(push 
	 (send (send *pr2* :copy-worldcoords) :rotate-vector
	       (float-vector (elt val 0) (elt val 1) (elt val 2)))
	 normal-list)))
    (setq *plane-normal-list* (reverse normal-list))))

(defun concated-cb ()
  (when (and *plgs-cb-flag* *cofs-cb-flag*)
    (let (centroid-list)
      (dotimes (i (length *vertices-list*))
	(let* ((c (send *tfl* :lookup-transform "/base_footprint" (format nil "/multi_plane_centroidoutput~0,2D" i) *cloud-stamp*)))
	  (print i)
	  (send c :transform (send *pr2* :copy-worldcoords) :world)
	  (push  (send c :copy-worldcoords) centroid-list)))
      (setq *cloud-centroid-list* (reverse centroid-list)))
    (draw-polygons)
    (setq *plgs-cb-flag* nil)
    (setq *cofs-cb-flag* nil)
    ))
    
(defun draw-polygons ()
  (send *irtviewer* :objects (list *pr2*))
  (let (polygons-list)
    (when (equal (length *vertices-list*) (length *plane-normal-list*))
      (dotimes (i (length *vertices-list*))
	(push (instance polygon :init :vertices (elt *vertices-list* i) :normal (elt *plane-normal-list* i))
	      polygons-list))
      (mapcar #'(lambda (plg ct)
		  (let* ((color (float-vector (random 1.0) (random 1.0) (random 1.0)))
			 (prism (make-prism (mapcar #'(lambda (p) (scale 1 p)) (send plg :vertices)) (send plg :normal)))
			 (sphere (make-sphere 20)))
		    (send sphere :move-to ct :world)
		    (send sphere :set-color :white)
		    (send prism :set-color color)
		    (objects sphere)
		    (objects prism)))
	      (reverse polygons-list) *cloud-centroid-list*)
      ;; (dolist (plg (reverse polygons-list))
      ;; 	(let* ((color (float-vector (random 1.0) (random 1.0) (random 1.0)))
      ;; 	       (prism (make-prism (mapcar #'(lambda (p) (scale 1 p)) (send plg :vertices)) (send plg :normal))))
      ;; 	  (send prism :set-color color)
      ;; 	  (objects prism)))
      (setq *polygons-list* (reverse polygons-list)))))


(defun choose-target-polygon ()
  (let (dist
	(min-dist 3000)
	pos-list
	iter 
	(cnt 0))
    (setq pos-list (mapcar #'(lambda (c) 
			       (let ((p (send c :worldpos)))
				 (float-vector (elt p 0) (elt p 1) 0.0)))
			       *cloud-centroid-list*))
    (dolist (pos pos-list)
      (setq dist (norm (v- pos (send *pr2* :worldpos))))
      (when (and (<= dist min-dist) (>= (abs (send (elt *polygons-list* cnt) :area)) (* 150 150)))
	(setq iter cnt)
	(setq min-dist dist))
	(setq cnt (+ cnt 1)))
      (when iter
	(elt *polygons-list* iter)))))

(defun find-closest-edge (plg pt) ;;polygon point
  (let (closest-edge 
	dist
	(min-dist (send (car (send plg :edges)) :distance pt)))
    (dolist (edge (send plg :edges))
      (setq dist (send edge :distance pt))
      (when (<= dist min-dist)
	  (setq closest-edge edge)
	  (setq min-dist dist)))
    closest-edge))

(defun find-candidate-grasp-points (edge pt)
  (let ((par (send edge :foot pt))
	pt-list)
    (cond ((<= par 0)
	   (dotimes (i 10)
	     (push (send edge :point (/ i 10.0)) pt-list))
	   (setq pt-list (cdr (reverse pt-list))))
	  ((>= par 1)
	   (dotimes (i 10)
	     (push (send edge :point (/ i 10.0)) pt-list))
	   (setq pt-list pt-list))
	  ((and (> par 0) (< par 1))
	   (dotimes (i 10)
	     (let ((pt+ (send edge :point (+ par (/ i 10.0))))
		   (pt- (send edge :point (- par (/ i 10.0)))))
	       (if (and (> (send edge :parameter pt+)  0) (< (send edge :parameter pt+) 1))
		   (push pt+ pt-list))
	       (if (and (> (send edge :parameter pt-) 0) (< (send edge :parameter pt-) 1))
		   (push pt- pt-list)))))
	  (t
	   ))
    pt-list))

(defun find-plane-grasp-coords ()
  (let* (l-closest-edge
	 l-tgt-pts
	 l-tgt-cds
	 r-closest-edge
	 r-tgt-pts
	 r-tgt-cds
	 (b4-av (send *pr2* :angle-vector))
	 (plg (choose-target-polygon)))
	
	 (send *pr2* :reset-pose)
	 (setq l-closest-edge (find-closest-edge plg (send *pr2* :larm :end-coords :worldpos)))
	 (setq r-closest-edge (find-closest-edge plg (send *pr2* :rarm :end-coords :worldpos)))

	 (setq l-tgt-pts (find-candidate-grasp-points l-closest-edge (send *pr2* :larm :end-coords :worldpos)))
	 (setq r-tgt-pts (find-candidate-grasp-points r-closest-edge (send *pr2* :rarm :end-coords :worldpos)))

	 (let ((rot (transpose (matrix (send l-closest-edge :binormal plg) (send l-closest-edge :direction) (send plg :normal))))
	       cds-list)
	   (dolist (l-tgt-pt l-tgt-pts)
	     (print l-tgt-pt)
	     (print rot)
	     (push (send (make-coords :pos l-tgt-pt :rot rot) :rotate (deg2rad 90) :x) cds-list))
	   (dolist (cds cds-list)
	     (when (send *pr2* :inverse-kinematics cds
			 :move-target (send *pr2* :larm :end-coords)
			 :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
			 :use-torso t
			 :rotational-axis t)
	       (setq l-tgt-cds cds)
	       (return))))

	 (let ((rot (transpose (matrix (send r-closest-edge :binormal plg) (send r-closest-edge :direction) (send plg :normal))))
	       cds-list)
	   (dolist (r-tgt-pt r-tgt-pts)
	     (push (send (make-coords :pos r-tgt-pt :rot rot) :rotate (deg2rad 90) :x) cds-list))
	   (dolist (cds cds-list)
	     (when (send *pr2* :inverse-kinematics cds
			 :move-target (send *pr2* :rarm :end-coords)
			 :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
			 :use-torso 0.0
			 :rotational-axis t)
	       (setq r-tgt-cds cds)
	       (return))))
	 
	 (send *pr2* :angle-vector b4-av)
	 (if (and l-tgt-cds r-tgt-cds)
	     (list l-tgt-cds r-tgt-cds)
	   nil))))

(defun pr2-grasp-plane ()
  (let ((tgt (find-plane-grasp-coords)))
    (send *pr2* :inverse-kinematics (car tgt)
	  :move-target (send *pr2* :larm :end-coords)
	  :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
	  :use-torso t
	  :rotational-axis t)
    (send *pr2* :inverse-kinematics (elt tgt 1)
	  :move-target (send *pr2* :rarm :end-coords)
	  :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	  :use-torso 0.0
	  :rotational-axis t)

    (send *pr2* :larm :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(-50 0 0)))
    (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(-50 0 0)))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)

    (send *pr2* :larm :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(50 0 0)))
    (send *pr2* :rarm :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(50 0 0)))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp)
))
    

(setq *tfl* (instance ros::transform-listener :init))
(ros::subscribe "/ground_object_segmentation/multi_plane_estimation/output/polygons"
		jsk_pcl_ros::PolygonArray
		#'polygons-array-cb)
(ros::subscribe "/ground_object_segmentation/multi_plane_estimation/output/coefficients"
		jsk_pcl_ros::ModelCoefficientsArray
		#'coefficients-array-cb)

(defun main()
  (ros::rate 5)
  (do-until-key
   (ros::spin-once)
   (concated-cb)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *irtviewer* :draw-objects :flush t)
   (ros::sleep)
   ))


(main)



(dolist (pt (find-candidate-grasp-points eg (send  *pr2* :larm :end-coords :worldpos)))
  (let ((c (make-cube 20 20 20 :pos pt)))
    (objects c)))