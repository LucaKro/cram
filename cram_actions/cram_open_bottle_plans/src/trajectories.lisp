;;;
;;; Copyright (c) 2020, Gayane Kazhoyan  <kazhoyan@cs.uni-bremen.de>
;;;                     Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                     Thomas Lipps     <tlipps@uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :ob-plans)

(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :open-bottle))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key
                                                           tool)

  (print "this is in bottle opening trajectories")
  (let* ((grasp grasp)
         (object
           objects-acted-on)
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (tool-object-type
           (desig:desig-prop-value tool :type))
         (bTo
           (man-int:get-object-transform object))

         
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         
         (oTg-pregrasps
           (man-int:get-object-type-to-gripper-pregrasp-transforms
            object-type object-name arm grasp location oTg-std))
         (pre-opening-transforms
           (calculate-pre-opening-trajectory object-type (list oTg-std) object-name tool-object-type))
         (opening-transforms
           (calculate-opening-trajectory object-type (copy-list pre-opening-transforms) object-name tool-object-type))
         )
    (print "Transform:")
    (print bTo)
    ;;(break)

    (mapcar (lambda (label transforms)
               (man-int:make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                              transforms)))
            '(:reach
              :grasping
              :pre-open
              :open)
            `(,oTg-pregrasps
              (,oTg-std)
              ,pre-opening-transforms
              ,opening-transforms
              ))))


(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :hold-bottle))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key)

  (let* ((object
           objects-acted-on)
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (bTo
           (man-int:get-object-transform object))
         (oTb
           (cram-tf:transform-stamped-inv bTo))
         (oTg-pregrasps
           (man-int:get-object-type-to-gripper-pregrasp-transforms
            object-type object-name arm grasp location oTg-std)))

    

    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                              transforms)))
            '(:reach
              :grasping)
            `(,oTg-pregrasps
              (,oTg-std)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; generics ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric get-object-type-robot-frame-open-reach-transform (object-type name arm grasp tool)
  (:documentation "Calculates the reach transform for the current opening action")
  (:method (object-type name arm grasp tool)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-open-reach-transform
                                      object-type name arm grasp tool)))

(defgeneric calculate-pre-opening-trajectory (object-type init-transform name tool &optional angle)
  (:documentation "Calculates the pre-opening transform for the current opening action")
  (:method (object-type init-transform name tool &optional (angle (cram-math:degrees->radians 10)))
    (man-int::call-with-specific-type #'calculate-pre-opening-trajectory
                                      object-type init-transform name tool angle)))

(defgeneric calculate-opening-trajectory (object-type init-transform name tool &optional angle)
  (:documentation "Calculates the opening transform for the current opening action")
  (:method (object-type init-transform name tool &optional (angle (cram-math:degrees->radians 10)))
    (man-int::call-with-specific-type #'calculate-opening-trajectory
                                      object-type init-transform name tool angle)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; by hand ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-open-reach-transform ((object-type (eql :removed-by-hand)) name arm (grasp (eql :top)) (tool (eql nil)))
  
  (let* ((z (cl-transforms:z
             (cl-bullet::bounding-box-dimensions
              (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (std-transf (cl-transforms-stamped:make-transform-stamped
                      "map" "base_footprint" 0
                      (cl-tf:make-3d-vector 0.0 0 0.0) (cl-tf:make-quaternion 0 0 0 1)));;1 0 -1 0)))
         (obj-rot (cl-tf:make-transform-stamped "base_footprint" "base_footprint" 0 (cl-tf:make-3d-vector 0.0 0 0.0)
                                        (cl-tf:orientation (btr:pose  (btr:object btr:*current-bullet-world* name))))))

    (cram-tf:apply-transform obj-rot std-transf)))

(defmethod calculate-pre-opening-trajectory ((object-type (eql :removed-by-hand)) init-transform name (tool (eql nil)) &optional (angle (cram-math:degrees->radians 10)))
  init-transform)

(defmethod calculate-opening-trajectory ((object-type (eql :removed-by-hand)) init-transform name (tool (eql nil)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 36)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (lift-offset (/ obj-height (* 2 times)))
         (result init-transform))
    (dotimes (n times)
      (let ((opening-transform (cram-tf:translate-transform-stamped
                                (cram-tf:rotate-transform-in-own-frame (car (last result)) :z (- angle))
                                :z-offset lift-offset)))

        (push opening-transform (cdr (last result)))))
    result))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,, caplifter ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-open-reach-transform ((object-type (eql :removed-by-caplifter)) name arm (grasp (eql :top)) (tool (eql :caplifter)))

  (let* ((rot (cl-tf:make-quaternion 0 0.523 0 0.852))  ;; 0.37 0.37 -0.603 0.603))
         (obj-radius (cl-transforms:x
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name))))))

    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0
     (cl-tf:make-3d-vector (- (+ 0.09 obj-radius)) 0 -0.135) rot)))

(defmethod calculate-pre-opening-trajectory ((object-type (eql :removed-by-caplifter)) init-transform name (tool (eql :caplifter)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 10)
         (result (last init-transform))
         (gTo (cram-tf:transform-stamped-inv (car result)))
         (new-transf (cram-tf:transform-stamped-inv
                      (cram-tf:copy-transform-stamped gTo
                                                      :translation (cl-tf:make-3d-vector 0 0.1 0)))))
         
    ;;(print obj-radius)

    (interpolate-transform (car (last init-transform)) new-transf times))
    
    ;; (dotimes (n (/ times 1))
    ;;   (let ((opening-pose (cram-tf:transform-stamped-inv
    ;;                        (cram-tf:translate-transform-stamped gTo
    ;;                                                             :x-offset (* 2 shift-offset)))))
    ;;     (push opening-pose (cdr (last result)))))
    
    ;; result)
  )

(defmethod calculate-opening-trajectory ((object-type (eql :removed-by-caplifter)) init-transform name (tool (eql :caplifter)) &optional (angle (cram-math:degrees->radians 90)))
  (let* ((times 10)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (init-transform (car (last init-transform)))
         (gTo (cram-tf:transform-stamped-inv init-transform))
         (init-transform-transl (cl-tf:translation init-transform))
         (axis (cond
                 ((< (abs (cl-tf:x init-transform-transl))
                     (abs (cl-tf:y init-transform-transl))) :x)
                 ((> (abs (cl-tf:x init-transform-transl))
                     (abs (cl-tf:y init-transform-transl))) :y)
                 (t (error "Case not covered"))))
                 
         (sign (determine-sign init-transform-transl))
         (angle (* sign (cram-math:degrees->radians 30)))
         (new-transf (cram-tf:transform-stamped-inv
                      (cram-tf:rotate-transform-in-own-frame
                       (cram-tf:copy-transform-stamped gTo
                                                       :translation (cl-tf:make-3d-vector 0 0.1 (/ obj-height 3))) 
                      ;; (cram-tf:translate-transform-stamped gTo
                      ;;                                                                     :y-offset -0.0323950855575664d0
                      ;;                                                                     :z-offset (- (* obj-height 1.5)))
                                                           ;; :x-offset 0.028
                                                           ;; :z-offset 0.052)
                       axis angle))))
                                                 
                    ;;:rotation (cl-tf:make-quaternion 0.707 -0.707 0 0))))
    ;; 0.5 0.5 -0.5 0.5))))
    ;; (print init-transform-transl)
    ;; (break)
    (interpolate-transform init-transform new-transf times)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cork ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-open-reach-transform ((object-type (eql :removed-by-corkscrew)) name arm (grasp (eql :top)) (tool (eql :corkscrew)))

  (let* ((z (cl-transforms:z
             (cl-bullet::bounding-box-dimensions
              (btr::aabb  (btr:object btr:*current-bullet-world* name))))))
    
    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0
     (cl-tf:make-3d-vector 0.0 0 0.0) (cl-tf:make-quaternion 1 0 -1 0))))

(defmethod calculate-pre-opening-trajectory ((object-type (eql :removed-by-corkscrew)) init-transform name (tool (eql :corkscrew)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 36)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (lift-offset (- (/ obj-height (* 2 times))))
         (result init-transform))
    (dotimes (n times)
      (let ((opening-transform (cram-tf:translate-transform-stamped
                           (cram-tf:rotate-transform-in-own-frame (car (last result)) :z  angle)
                           :z-offset lift-offset)))

        (push opening-transform (cdr (last result)))))
    
    result))
     

(defmethod calculate-opening-trajectory ((object-type (eql :removed-by-corkscrew)) init-transform name (tool (eql :corkscrew)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 18)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (lift-offset (+ obj-height 0.01))
         (result (last init-transform))
         (end-transf (cram-tf:translate-transform-stamped (car result)
                                                             :z-offset lift-offset)))
    (interpolate-transform (car result) end-transf times)))

;; (dotimes (n times)
;;       (let ((opening-transform (cram-tf:translate-transform-stamped (car (last result))
;;                                                                :z-offset (- lift-offset))))
;;         (push opening-transform (cdr (last result)))))
    
;;     result))


;; (new-transf (cram-tf:transform-stamped-inv
;;                       (cram-tf:rotate-transform-in-own-frame
;;                        (cram-tf:copy-transform-stamped gTo
;;                                                        :translation (cl-tf:make-3d-vector 0 0.1 (/ obj-height 3)))
;;                        axis angle))))
;;     (interpolate-transform init-transform new-transf 10)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; other ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun dot-product (matrix-a matrix-b)
  (let* ((rows-a (array-dimensions matrix-a))
         (cols-a (second rows-a))
         (rows-b (array-dimensions matrix-b))
         (cols-b (second rows-b)))
    (if (= cols-a (first rows-b))
        (let* ((result (make-array (list (first rows-a) cols-b) :initial-element 0)))
          (dotimes (i (first rows-a))
            (dotimes (j cols-b)
              (let ((sum 0))
                (dotimes (k cols-a)
                  (incf sum (* (aref matrix-a i k) (aref matrix-b k j))))
                (setf (aref result i j) sum))))
          result))))

(defun slerp-q (q1 q2 t-val)
  (let ((dot-prod (min 1.0 (max -1.0 (cl-tf:q-dot q2 q1))))
        result
        theta
        sin-theta)
    (when (< dot-prod 0.0)
      (setf q2 (cl-tf:q-inv q2))
      (setf dot-prod (- dot-prod)))
    (setf theta (acos dot-prod))
    (setf sin-theta (sin theta))


    (cond
      ((< sin-theta 0.00001) (setf result (cl-tf:q+ (cl-tf:q-scale q1 (- 1 t-val))
                                                    (cl-tf:q-scale q2 t-val))))
      (t (setf result (cl-tf:q-scale (cl-tf:q+ (cl-tf:q-scale q1 (sin (* (- 1 t-val) theta)))
                                               (cl-tf:q-scale q2 (sin (* t-val theta))))
                                     (/ 1 sin-theta)))))
    result))

(defun slerp-p (p1 p2 t-val)
  (let ((dot-prod (min 1.0 (max -1.0 (cl-tf:dot-product p2 p1))))
        result
        theta
        sin-theta)
    (when (< dot-prod 0.0001)
      (setf p2 (cl-tf:v-inv p2))
      (setf dot-prod (- dot-prod)))
    
    (setf theta (acos dot-prod))
    (setf sin-theta (sin theta))

    (cond
      ((< sin-theta 0.00001) (setf result (cl-tf:v+ (cl-tf:v* p1 (- 1 t-val))
                                                    (cl-tf:v* p2 t-val))))
      (t (setf result (cl-tf:v* (cl-tf:v+ (cl-tf:v* p1 (sin (* (- 1 t-val) theta)))
                                               (cl-tf:v* p2 (sin (* t-val theta))))
                                     (/ 1 sin-theta)))))
    result))

(defun lerp-p (p1 p2 t-val)
  (cl-tf:v+ (cl-tf:v* p1 (- 1 t-val)) (cl-tf:v* p2 t-val)))

(defun interpolate-pose (start-pose end-pose steps)
  (let (interp
        (step-size (/ 1.0 (- steps 1))))
    (loop for i from 0 to steps
          do
             (let* ((t-val (* i step-size))
                    (transl (slerp-p (cl-tf:origin start-pose) (cl-tf:origin end-pose) t-val))
                    (rot (slerp-q (cl-tf:orientation start-pose) (cl-tf:orientation end-pose) t-val))
                    (pose (cl-tf:make-pose-stamped "map" 0 transl rot)))
               (unless (> t-val 1)
                 (push pose interp))))
    (reverse interp)))

(defun interpolate-transform (start-transform end-transform steps)
  (let (interp
        (step-size (/ 1.0 (- steps 1)))
        (parent (cl-tf:frame-id start-transform))
        (child (cl-tf:child-frame-id start-transform))
        (stamp (cl-tf:stamp start-transform)))
    (loop for i from 0 to steps
          do
             (let* ((t-val (* i step-size))
                    (transl (lerp-p (cl-tf:translation start-transform) (cl-tf:translation end-transform) t-val))
                    (rot (slerp-q (cl-tf:rotation start-transform) (cl-tf:rotation end-transform) t-val))
                    (transform (cl-tf:make-transform-stamped parent child stamp transl rot)))
               (unless (> t-val 1)
                 (push transform interp))))
    (reverse interp)))

(defun determine-sign (vec)
  (let ((diff  (- (cl-tf:x vec)
                  (cl-tf:y vec)
                  (cl-tf:z vec))))
    (cond
      ((< diff 0) -1)
      (t 1))))


;; @author 
(defun rotate-once-pose (pose angle axis)
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
                  (cl-tf:normalize
                   (cl-transforms:q*
                    (cl-transforms:axis-angle->quaternion
                     (case axis
                       (:x (cl-transforms:make-3d-vector 1 0 0))
                       (:y (cl-transforms:make-3d-vector 0 1 0))
                       (:z (cl-transforms:make-3d-vector 0 0 1))
                       (t (error "in ROTATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
                     angle)
                    pose-orientation)))))