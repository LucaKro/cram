;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-manipulation-knowledge)

(defclass object-shape-data-mixin ()
  ((shape-type :initarg :shape-type :reader shape-type
               :documentation "A symbol indicating the type of the shape.")
   (dimensions :initarg :dimensions :reader dimensions
               :documentation "A 3D-VECTOR containing the dimensions
               of the object.")))

(defclass object-mesh-data-mixin ()
  ((vertices :initarg :vertices :reader vertices
             :documentation "Contains a sequence of vertices of type
             CL-TRANSFORMS:POINT.")
   (faces :initarg :faces :reader faces
          :documentation "Contains a sequence of lists of vertex
          indices."))
  (:documentation "Mixin class to be used in a child class of
  OBJECT-DESIGNATOR-DATA to represent objects for which a mesh is
  known."))

(defclass object-point-data-mixin ()
  ((points :initarg :points :reader points :reader vertices
           :documentation "A sequence of points of type
           CL-TRANSFORMS:POINT representing a point cloud."))
  (:documentation "Mixin class to be used in a child class of
  OBJECT-DESIGNATOR-DATA to represent objects for which only point
  clouds are known."))

(defgeneric get-shape-message (object-designator-data)
  (:documentation "Returns an instance of arm_navigation_msgs/Shape
  representing the object referenced by OBJECT-DESIGNATOR-DATA or NIL
  if no shape can be generated.")
  (:method ((data desig:object-designator-data))
    nil)

  (:method ((data object-shape-data-mixin))
    (with-slots (shape-type dimensions) data 
      (roslisp:make-message
       "arm_navigation_msgs/Shape"
       :type (roslisp-msg-protocol:symbol-code
              'arm_navigation_msgs-msg:shape shape-type)
       :dimensions (vector
                    (cl-transforms:x dimensions)
                    (cl-transforms:y dimensions)
                    (cl-transforms:z dimensions)))))
  
  (:method ((data object-mesh-data-mixin))
    (with-slots (vertices faces) data
      (roslisp:make-message
       "arm_navigation_msgs/Shape"
       :type (roslisp-msg-protocol:symbol-code
              'arm_navigation_msgs-msg:shape :mesh)
       :triangles (car
                   (reduce (lambda (result triangle)
                             (destructuring-bind (vector . index)
                                 result
                               (assert (eql (list-length triangle) 3))
                               (loop for point in triangle
                                     for i from index
                                     do (setf (aref vector i) point)
                                     finally (return (cons vector i)))))
                           faces :initial-value (cons (make-array (* 3 (length faces))) 0)))
       :vertices (map 'vector (lambda (point)
                                (roslisp:make-msg
                                 "geometry_msgs/Point"
                                 :x (cl-transforms:x point)
                                 :y (cl-transforms:y point)
                                 :z (cl-transforms:z point)))
                      vertices))))

  (:method ((data object-point-data-mixin))
    (with-slots (points) data
      (multiple-value-bind (size center)
          (physics-utils:calculate-aabb points)
        (roslisp:make-msg
         "arm_navigation_msgs/Shape"
         :type (roslisp-msg-protocol:symbol-code
                'arm_navigation_msgs-msg:shape :box)
         :dimensions (vector
                      (- (cl-transforms:x size) (cl-transforms:x center))
                      (- (cl-transforms:y size) (cl-transforms:y center))
                      (- (cl-transforms:z size) (cl-transforms:z center))))))))

