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
		    (send *irtviewer* :objects sphere)
		    (send *irtviewer* :objects prism)))
	      (reverse polygons-list) *cloud-centroid-list*)
      (setq *polygons-list* (reverse polygons-list)))))


(defun sort-polygons-by-dist-from (plgs pt)
  (let ((sorted-plgs (copy-object plgs)))
    (sort sorted-plgs #'(lambda (plg1 plg2)
			   (if (<= (norm (v- (elt (send plg1 :centroid) 1) pt)) (norm (v- (elt (send plg2 :centroid) 1) pt)))
			       t)))
    sorted-plgs))

(defun sort-edge-by-dist-from (plg pt)
  (let ((sorted-edges (copy-object (send plg :edges))))
    (sort sorted-edges #'(lambda (e1 e2) 
			   (if (<= (send e1 :distance pt) (send e2 :distance pt))
			       t)))
    sorted-edges))


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

(defun calc-grasp-motion-from-edge (plg edge use-arm use-torso)
  (let ((tgt-pts (find-candidate-grasp-points edge (send *pr2* use-arm :end-coords :worldpos)))
	(rot (transpose (matrix (v* (send edge :direction) (send plg :normal)) (send edge :direction) (send plg :normal))))
	cds-list
	avs
	pre-cds 
	tgt-cds)
    
    (setq pt-l tgt-pts)
    (dolist (tgt-pt tgt-pts)
      (push (send (make-coords :pos tgt-pt :rot rot) :rotate (deg2rad 90) :x) cds-list))
    
    (setq cds-l cds-list)

    (dolist (cds (reverse cds-list))
      (setq avs nil)
      (setq pre-cds (send cds :translate #f(-50 0 0)))
      (unless (send *pr2* :inverse-kinematics pre-cds
		    :move-target (send *pr2* use-arm :end-coords)
		    :link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
		    :use-torso (if use-torso t 0.0)
		    :rotational-axis t
		    :debug-view nil)
	(send pre-cds :rotate (deg2rad -180) :x))
      (if (send *pr2* :inverse-kinematics pre-cds
		:move-target (send *pr2* use-arm :end-coords)
		:link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
		:use-torso (if use-torso t 0.0)
		:rotational-axis t
		:debug-view nil)
	  (progn
	    (push (send *pr2* :angle-vector) avs)
	    (setq tgt-cds (send pre-cds :translate #f(80 0 0)))
	    (when (send *pr2* :inverse-kinematics tgt-cds
			:move-target (send *pr2* use-arm :end-coords)
			:link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
			:use-torso (if use-torso t 0.0)
			:rotational-axis t
			:debug-view nil)
	      (push (send *pr2* :angle-vector) avs)
		(return)))))
    (if (equal (length avs) 2)
	(progn
	  (let ((c (make-cube 30 30 30 :coords (send tgt-cds :copy-worldcoords))))
	    (send c :set-color (if (equal use-arm :larm) :red :green))
	    (send *irtviewer* :objects c)
	    (send *irtviewer* :draw-objects :flush t))
	  (reverse avs))
      nil)))


  

(defun generate-grasp-plane-motions ()
  (let* (l-closest-edge-seq
	 l-tgt-pts
	 r-closest-edge-seq
	 r-tgt-pts
	 avs1 avs2
	 final-avs
	 (b4-av (send *pr2* :angle-vector))
	 l-plg
	 r-plg)
    
    (send *pr2* :reset-pose)

	 (dolist (plg (sort-polygons-by-dist-from *polygons-list* (send *pr2* :larm :end-coords :worldpos)))
	   (when (>= (abs (send plg :area)) (* 150 150))
	     (setq l-plg plg)
	     (return)))
	 (dolist (plg (sort-polygons-by-dist-from *polygons-list* (send *pr2* :rarm :end-coords :worldpos)))
	   (when (>= (abs (send plg :area)) (* 150 150))
	     (setq r-plg plg)
	     (return)))
	 (unless (and l-plg r-plg)
	   (print "No graspable plane! Abort")
	   (return-from find-plane-grasp-coords))
		  
	 
	 (setq l-closest-edge-seq (sort-edge-by-dist-from l-plg (send *pr2* :larm :end-coords :worldpos)))
	 (setq r-closest-edge-seq (sort-edge-by-dist-from r-plg (send *pr2* :rarm :end-coords :worldpos)))

	 (if (>= (elt (elt (send l-plg :centroid) 1) 2) (elt (elt (send r-plg :centroid) 1) 2)) ;;if z of l-plg > z of r-plg
	     (progn 
	       (dolist (edge l-closest-edge-seq)
		 (setq avs1 (calc-grasp-motion-from-edge l-plg edge :larm t))
		 (when avs1 (return)))
	       (dolist (edge r-closest-edge-seq)
		 (setq avs2 (calc-grasp-motion-from-edge r-plg edge :rarm nil))
		 (when avs2 (return))))
	   (progn 
	     (dolist (edge r-closest-edge-seq)
	       (setq avs1 (calc-grasp-motion-from-edge r-plg edge :rarm t))
	       (when avs1 (return)))
	     (dolist (edge l-closest-edge-seq)
	       (setq avs2 (calc-grasp-motion-from-edge l-plg edge :larm nil))
	       (when avs2 (return)))))

	 ;;(send *pr2* :angle-vector b4-av)
	 (setq a1 avs1)
	 (setq a2 avs2)
	 (if (and avs1 avs2)
	     (progn
	       (setq final-avs (list (v- (v+ (elt avs1 0) (elt avs2 0)) (elt avs1 1)) (elt avs2 1)))
	       final-avs)
	   nil)))

(defun pr2-grasp-plane ()
  (let* ((avs (generate-grasp-plane-motions))
	 (pre-av (car avs))
	 (tgt-av (elt avs 1)))
    (send *ri* :angle-vector pre-av 5000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)
    (send *ri* :angle-vector tgt-av 1000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :larm :gain 0.1)
    (send *ri* :start-grasp :rarm :gain 0.1)))
    

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



;; (dolist (pt (find-candidate-grasp-points eg (send  *pr2* :larm :end-coords :worldpos)))
;;   (let ((c (make-cube 20 20 20 :pos pt)))
;;     (objects c)))