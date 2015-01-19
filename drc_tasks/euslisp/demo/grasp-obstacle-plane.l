(ros::roseus-add-msgs "jsk_pcl_ros")
(require "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")

(setq *vertices-list-org* nil)
(setq *plane-normal-list* nil)
(setq *polygons-list* nil)
(setq *cloud-stamp* nil)
(setq *cloud-centroid-list* nil)

(setq *plgs-cb-flag* nil)
(setq *cofs-cb-flag* nil)

(setq *grasp-motions-avs* nil)

(unless (boundp '*pr2*)
  (pr2))
(unless (boundp '*ri*)
  (setq *ri* (instance pr2-interface :init)))
(unless (boundp '*irtviewer*)
  (make-irtviewer))

(defun polygons-array-cb (msg)
  (ros::ros-info "polygons callback")
  (setq *plgs-cb-flag* t)
  (setq *cloud-stamp* (send msg :header :stamp))
  (let ((plg-list (send msg :polygons))
	centroid-list)
    (setq *vertices-list-org* (mapcar #'(lambda (plg)
				       (let ((pts (send (send plg :polygon) :points))
					     (pts-list))
					 (dolist (pt pts)
					   (push 
					    (send (send *pr2* :copy-worldcoords) :transform-vector 
						  (scale 1000 (float-vector (send pt :x) (send pt :y) (send pt :z))))
					    pts-list))
					 (reverse pts-list)))
				      plg-list))
    (dotimes (i (length *vertices-list-org*))
      (let* ((c (send *tfl* :lookup-transform "/base_footprint" (format nil "/multi_plane_centroidoutput~0,2D" i) *cloud-stamp*)))
	(when c
	  (send c :transform (send *pr2* :copy-worldcoords) :world)
	  (push  (send c :copy-worldcoords) centroid-list))))
    (setq *cloud-centroid-list* (reverse centroid-list))
    (init-eus-polygons)))
    
(defun init-eus-polygons ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *irtviewer* :objects (list *pr2*))
  (let (polygons-list)
    ;;shift the detected polygons a little considering the thickness
      (dotimes (i (length *vertices-list-org*))
	(let ((plg-temp (instance polygon :init :vertices (elt *vertices-list-org* i)))
	      plg v)
	  (if (>= (v. (matrix-column (send *pr2* :head :end-coords :worldrot) 2) (send plg-temp :normal)) 0)
	      (setq v (send plg-temp :normal))
	    (setq v (scale -1.0 (send plg-temp :normal))))
	  (setq plg (instance polygon :init :vertices (mapcar #'(lambda (pt) (v+ pt (scale 35 v))) (send plg-temp :vertices))))
	  (push plg polygons-list)))

      (mapcar #'(lambda (plg ct)
		  (send plg :reset-normal)
		  (let* ((color (float-vector (random 1.0) (random 1.0) (random 1.0)))
			 (prism (make-prism (mapcar #'(lambda (p) (scale 1 p)) (send plg :vertices)) (send plg :normal)))
			 (sphere (make-sphere 20)))
		    (send sphere :move-to ct :world)
		    (send sphere :set-color :white)
		    (send prism :set-color color)
		    (send *irtviewer* :objects sphere)
		    (send *irtviewer* :objects prism)))
	      (reverse polygons-list)
	      (if (equal (length *cloud-centroid-list*) (length polygons-list))
		  (progn (print "check1")
			 *cloud-centroid-list*)
		(progn (print "check2")
		       (setq plg-l polygons-list)
		       (mapcar #'(lambda (p) (make-coords :pos(elt (send p :centroid) 1))) (reverse polygons-list)))))
      (send *irtviewer* :draw-objects :flush t)
      (setq *polygons-list* (reverse polygons-list))))


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
	(n (round (/ (send edge :length) 50))) ;;sample a point every 5cm 
	pt-list)
    (when (>= n 0)
      (dotimes (i 9)
	(push (send edge :point (/ (+ i 1) 10.0)) pt-list))
    (sort pt-list #'(lambda (p1 p2)
		      (if (<= (norm (v- p1 pt)) (norm (v- p2 pt)))
			  t))))
    (subseq pt-list 0 n)))

(defun calc-grasp-motion-from-edge (plg edge use-arm use-torso)
  (let ((tgt-pts (find-candidate-grasp-points edge (send *pr2* use-arm :end-coords :worldpos)))
	(rot (transpose (matrix (normalize-vector (v* (send plg :normal) (send edge :direction))) (scale -1.0 (send edge :direction)) (send plg :normal))))
	cds-list
	avs
	pre-cds 
	tgt-cds)
    
    (unless tgt-pts
      (return-from calc-grasp-motion-from-edge nil))
    
    (setq pt-l tgt-pts)
    (dolist (tgt-pt tgt-pts)
      (let ((c (send (make-coords :pos tgt-pt :rot rot) :copy-worldcoords)))
	(send c :rotate (deg2rad 90.0) :x)
	(push c cds-list)))

    (setq cds-l cds-list)
     
    (dolist (cds (reverse cds-list))
      (setq avs nil)
      ;;make-sure two grppier would not be too close
      (when (or use-torso (and (not use-torso) (>= (norm (v- (send cds :worldpos) (send *pr2* (if (equal use-arm :larm) :rarm :larm) :end-coords :worldpos))) 200)))
	(objects (send cds :copy-worldcoords))
	(setq pre-cds (send cds :translate #f(-50 0 0)))
	(unless (send *pr2* :inverse-kinematics pre-cds
		      :move-target (send *pr2* use-arm :end-coords)
		      :link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
		      :use-torso (if use-torso t 0.0)
		      ;;:use-base (if use-torso t nil)
		      :rotational-axis t
		      :look-at-target t
		      :debug-view nil)
	  (send pre-cds :rotate (deg2rad -180) :x))
	(if (send *pr2* :inverse-kinematics pre-cds
		  :move-target (send *pr2* use-arm :end-coords)
		  :link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
		  :use-torso (if use-torso t 0.0)
		;;:use-base (if use-torso t nil)
		:rotational-axis t
		:look-at-target t
		:debug-view nil)
	  (progn
	    (push (send *pr2* :angle-vector) avs)
	    (setq tgt-cds (send pre-cds :translate #f(100 0 0)))
	    (when (send *pr2* :inverse-kinematics tgt-cds
			:move-target (send *pr2* use-arm :end-coords)
			:link-list (send *pr2* :link-list (send *pr2* use-arm :end-coords :parent))
			:use-torso (if use-torso t 0.0)
			:rotational-axis t
			:look-at-target t
			:debug-view nil)
	      (push (send *pr2* :angle-vector) avs)
	      (return))))
	))
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
	   (when (>= (abs (send plg :area)) (* 200 200))
	     (setq l-plg plg)
	     (return)))
	 (dolist (plg (sort-polygons-by-dist-from *polygons-list* (send *pr2* :rarm :end-coords :worldpos)))
	   (when (>= (abs (send plg :area)) (* 200 200))
	     (setq r-plg plg)
	     (return)))
	 (unless (and l-plg r-plg)
	   (print "No graspable plane! Abort")
	   (return-from generate-grasp-plane-motions))
		  
	 
	 (setq l-closest-edge-seq (sort-edge-by-dist-from l-plg (send *pr2* :larm :end-coords :worldpos)))
	 (setq r-closest-edge-seq (sort-edge-by-dist-from r-plg (send *pr2* :rarm :end-coords :worldpos)))
	 
	 (let ((cnt 0))
	   (if (>= (elt (elt (send l-plg :centroid) 1) 2) (elt (elt (send r-plg :centroid) 1) 2)) ;;if z of l-plg > z of r-plg
	       (progn 
		 (dolist (edge (subseq l-closest-edge-seq 0 10))
		   (when (> (send edge :length) 50.0)
		     (setq avs1 (calc-grasp-motion-from-edge l-plg edge :larm t))
		     (incf cnt))
		   (when (or (>= cnt 3) avs1) (return)))
		 (when avs1
		   (dolist (edge (subseq r-closest-edge-seq 0 10))
		     (if (> (send edge :length) 50.0)
			 (setq avs2 (calc-grasp-motion-from-edge r-plg edge :rarm nil)))
		     (when avs2 (return)))))
	     (progn 
	       (dolist (edge (subseq r-closest-edge-seq 0 10))
		 (when (> (send edge :length) 50.0)
		   (setq avs1 (calc-grasp-motion-from-edge r-plg edge :rarm t))
		   (incf cnt))
		 (when (or (>= cnt 3) avs1) (return)))
	       (when avs1 
		 (dolist (edge (subseq l-closest-edge-seq 0 10))
		   (if (> (send edge :length) 50.0)
		       (setq avs2 (calc-grasp-motion-from-edge l-plg edge :larm nil)))
		   (when avs2 (return)))))))

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
	 pre-av tgt-av moveit-res)
    (setq *grasp-motions-avs* avs)
    (when avs
      (setq pre-av (car avs))
      (setq tgt-av (elt avs 1))
      (setq moveit-res
      	   (send *ri* :angle-vector-motion-plan pre-av :move-arm :arms :use-torso t))
      (if moveit-res
	  (progn
	    (warn "Move-it Succeeded")
	    (send *ri* :wait-interpolation)
	    (send *ri* :angle-vector pre-av 500)
	    )
	(progn
	  (warn "Move-it Failed")
	  (send *ri* :angle-vector pre-av 5000)))

	  (send *ri* :wait-interpolation)
	  (send *ri* :stop-grasp)
	  (unix:usleep (* 1000 1000))
	  (send *ri* :angle-vector tgt-av 3000)
	  (send *ri* :wait-interpolation)
	  (send *ri* :start-grasp :larm :gain 0.1)
	  (send *ri* :start-grasp :rarm :gain 0.1))))

  (defun pr2-release-plane ()
  (let ((pre-av (car *grasp-motions-avs*)))
    (when pre-av
      (send *ri* :stop-grasp)
      (unix:usleep (* 1000 1000))
      (send *ri* :angle-vector pre-av 3000)
      (send *ri* :wait-interpolation)
      (let ((a (send *pr2* :head :neck-p :joint-angle)))
	(send *pr2* :reset-pose)
	(send *pr2* :head :neck-p :joint-angle a))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation))))
	
	
  

(setq *tfl* (instance ros::transform-listener :init))
(ros::subscribe "/ground_object_segmentation/multi_plane_estimate/output_refined_polygon"
		jsk_pcl_ros::PolygonArray
		#'polygons-array-cb)

;; (defun main()
;;   (ros::rate 5)
;;   (do-until-key
;;    (ros::spin-once)
;;    (concated-cb)
;;    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;    (send *irtviewer* :draw-objects :flush t)
;;    (ros::sleep)
;;    ))


