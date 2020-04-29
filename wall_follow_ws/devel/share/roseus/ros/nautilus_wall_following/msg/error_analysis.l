;; Auto-generated. Do not edit!


(when (boundp 'nautilus_wall_following::error_analysis)
  (if (not (find-package "NAUTILUS_WALL_FOLLOWING"))
    (make-package "NAUTILUS_WALL_FOLLOWING"))
  (shadow 'error_analysis (find-package "NAUTILUS_WALL_FOLLOWING")))
(unless (find-package "NAUTILUS_WALL_FOLLOWING::ERROR_ANALYSIS")
  (make-package "NAUTILUS_WALL_FOLLOWING::ERROR_ANALYSIS"))

(in-package "ROS")
;;//! \htmlinclude error_analysis.msg.html


(defclass nautilus_wall_following::error_analysis
  :super ros::object
  :slots (_avg_error _max_error ))

(defmethod nautilus_wall_following::error_analysis
  (:init
   (&key
    ((:avg_error __avg_error) 0.0)
    ((:max_error __max_error) 0.0)
    )
   (send-super :init)
   (setq _avg_error (float __avg_error))
   (setq _max_error (float __max_error))
   self)
  (:avg_error
   (&optional __avg_error)
   (if __avg_error (setq _avg_error __avg_error)) _avg_error)
  (:max_error
   (&optional __max_error)
   (if __max_error (setq _max_error __max_error)) _max_error)
  (:serialization-length
   ()
   (+
    ;; float32 _avg_error
    4
    ;; float32 _max_error
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _avg_error
       (sys::poke _avg_error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_error
       (sys::poke _max_error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _avg_error
     (setq _avg_error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_error
     (setq _max_error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get nautilus_wall_following::error_analysis :md5sum-) "02c9ccf4f42efdb87083aa4f2e65fcfe")
(setf (get nautilus_wall_following::error_analysis :datatype-) "nautilus_wall_following/error_analysis")
(setf (get nautilus_wall_following::error_analysis :definition-)
      "
float32 avg_error
float32 max_error

")



(provide :nautilus_wall_following/error_analysis "02c9ccf4f42efdb87083aa4f2e65fcfe")


