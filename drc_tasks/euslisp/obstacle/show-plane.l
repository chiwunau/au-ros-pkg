(ros::load-ros-manifest "drc_tasks")
(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus "subscribe_plane_to_eus")

(setq *vertices-array* nil)
(setq *plane-normal-array* nil)
(setq hoge nil)

(setq *plgs-cb-flag* nil)
(setq *cofs-cb-flag* nil)

(defun polygons-array-cb (msg)
  (setq *plgs-cb-flag* t)
  (let ((plg-list (send msg :polygons)))
    (setq *vertices-array* (mapcar #'(lambda (plg)
				       (let ((pts (send (send plg :polygon) :points))
					     (pts-list))
					 (dolist (pt pts)
					   (push (float-vector (send pt :x) (send pt :y) (send pt :z)) pts-list))
					 (reverse pts-list)))
				   plg-list))))

(defun coefficients-array-cb (msg)
    (setq *cofs-cb-flag* t)
  (let ((cof-list (send msg :coefficients))
	normal-array)
    (dolist (cof cof-list)
      (let ((val (send cof :values)))
	(push (float-vector (elt val 0) (elt val 1) (elt val 2)) normal-array)))
    (setq *plane-normal-array* (reverse normal-array))))

(defun concated-cb ()
  (when (and *plgs-cb-flag* *cofs-cb-flag*)
    (draw-polygons)
    (setq *plgs-cb-flag* nil)
    (setq *cofs-cb-flag* nil)
    ))
    




(setq *polygons-array* nil)

(defun draw-polygons ()
  (let (polygons-array)
    (when (equal (length *vertices-array*) (length *plane-normal-array*))
      (dotimes (i (length *vertices-array*))
	(push (instance polygon :init :vertices (elt *vertices-array* i) :normal (elt *plane-normal-array* i))
	      polygons-array))
      (dolist (plg (reverse polygons-array))
	(let* ((color (float-vector (random 1.0) (random 1.0) (random 1.0)))
	       (prism (make-prism (mapcar #'(lambda (p) (scale 1000 p)) (send plg :vertices)) (send plg :normal))))
	  (send prism :set-color color)
	  (objects prism)))
      (setq *polygons-array* (reverse polygons-array)))))




(defun main()
  (ros::subscribe "/ground_object_segmentation/multi_plane_estimation/output/polygons"
		  jsk_pcl_ros::PolygonArray
		  #'polygons-array-cb)
  (ros::subscribe "/ground_object_segmentation/multi_plane_estimation/output/coefficients"
		  jsk_pcl_ros::ModelCoefficientsArray
		  #'coefficients-array-cb)
  (ros::rate 5)
  (do-until-key
   (ros::spin-once)
   (concated-cb)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *irtviewer* :draw-objects :flush t)
   (ros::sleep)
   ))


(main)

