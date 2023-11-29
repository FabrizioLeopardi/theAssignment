;; Auto-generated. Do not edit!


(when (boundp 'assignment1_EXP::Marker)
  (if (not (find-package "ASSIGNMENT1_EXP"))
    (make-package "ASSIGNMENT1_EXP"))
  (shadow 'Marker (find-package "ASSIGNMENT1_EXP")))
(unless (find-package "ASSIGNMENT1_EXP::MARKER")
  (make-package "ASSIGNMENT1_EXP::MARKER"))

(in-package "ROS")
;;//! \htmlinclude Marker.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass assignment1_EXP::Marker
  :super ros::object
  :slots (_id _corners ))

(defmethod assignment1_EXP::Marker
  (:init
   (&key
    ((:id __id) 0)
    ((:corners __corners) ())
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _corners __corners)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:corners
   (&rest __corners)
   (if (keywordp (car __corners))
       (send* _corners __corners)
     (progn
       (if __corners (setq _corners (car __corners)))
       _corners)))
  (:serialization-length
   ()
   (+
    ;; int64 _id
    8
    ;; geometry_msgs/Point[] _corners
    (apply #'+ (send-all _corners :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; geometry_msgs/Point[] _corners
     (write-long (length _corners) s)
     (dolist (elem _corners)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; geometry_msgs/Point[] _corners
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _corners (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _corners)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get assignment1_EXP::Marker :md5sum-) "29e916ca9398fbd7b2424ca3f887845d")
(setf (get assignment1_EXP::Marker :datatype-) "assignment1_EXP/Marker")
(setf (get assignment1_EXP::Marker :definition-)
      "int64 id
geometry_msgs/Point[] corners
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :assignment1_EXP/Marker "29e916ca9398fbd7b2424ca3f887845d")


