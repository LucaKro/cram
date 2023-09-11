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
  (let* ((grasp :top)
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

         ;;;;vvvvvvvvv;;;;;;
         (to-T-to-offset
           (get-object-type-robot-frame-open-reach-transform
            object-type object-name arm grasp tool-object-type))
         
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (oTg-pregrasps
           (man-int:get-object-type-to-gripper-pregrasp-transforms
            object-type object-name arm grasp location oTg-std))

         (reach-pose
           ;; (alexandria:curry #'man-int:calculate-gripper-pose-in-map
           ;;  bTo arm oTg-pregrasps)
           (cl-tf:copy-pose-stamped 
            (man-int:calculate-gripper-pose-in-base
              (cram-tf:apply-transform
               (cram-tf:copy-transform-stamped 
                to-T-to-offset
                :rotation (cl-tf:rotation to-T-to-offset)) ;; (cl-tf:make-identity-rotation))
               bTo)
              arm (car oTg-pregrasps))
            :orientation 
            (cl-tf:rotation to-T-to-offset))
            )

         (grasp-pose
           (cl-tf:copy-pose-stamped 
            (man-int:calculate-gripper-pose-in-base
              (cram-tf:apply-transform
               (cram-tf:copy-transform-stamped 
                to-T-to-offset
                :rotation (cl-tf:make-identity-rotation))
               bTo)
              arm oTg-std)
            :orientation 
            (cl-tf:rotation to-T-to-offset)))
         (pre-opening-poses
           (calculate-pre-opening-trajectory object-type (list grasp-pose) object-name tool-object-type))
         (opening-poses
           (calculate-opening-trajectory object-type (copy-list pre-opening-poses) object-name tool-object-type)))
    (print "Pose:")
    (print reach-pose)
    ;;(break)
    (mapcar (lambda (label poses-in-base)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar 
                       (lambda (pose-in-base)
                         (let ((mTb (cram-tf:pose->transform-stamped
                                     cram-tf:*fixed-frame*
                                     cram-tf:*robot-base-frame*
                                     0.0
                                     (cram-tf:robot-current-pose)))
                               (bTg-std
                                 (cram-tf:pose-stamped->transform-stamped
                                  pose-in-base
                                  (cl-tf:child-frame-id bTo))))
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg-std))))
                       poses-in-base)))
            
            `(:reach
              :grasping
              :pre-open
              :open)
            `((,reach-pose)
              (,grasp-pose)
              ,pre-opening-poses
              ,opening-poses))))


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
  (:documentation "Calculates the reach pose for the current opening action")
  (:method (object-type name arm grasp tool)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-open-reach-transform
                                      object-type name arm grasp tool)))

(defgeneric calculate-pre-opening-trajectory (object-type init-pose name tool &optional angle)
  (:documentation "Calculates the pre-opening pose for the current opening action")
  (:method (object-type init-pose name tool &optional (angle (cram-math:degrees->radians 10)))
    (man-int::call-with-specific-type #'calculate-pre-opening-trajectory
                                      object-type init-pose name tool angle)))

(defgeneric calculate-opening-trajectory (object-type init-pose name tool &optional angle)
  (:documentation "Calculates the opening pose for the current opening action")
  (:method (object-type init-pose name tool &optional (angle (cram-math:degrees->radians 10)))
    (man-int::call-with-specific-type #'calculate-opening-trajectory
                                      object-type init-pose name tool angle)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; by hand ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-open-reach-transform ((object-type (eql :removed-by-hand)) name arm (grasp (eql :top)) (tool (eql nil)))
  
  (let* ((z (cl-transforms:z
             (cl-bullet::bounding-box-dimensions
              (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (std-transf (cl-transforms-stamped:make-transform-stamped
                      "map" "base_footprint" 0
                      (cl-tf:make-3d-vector 0.0 0 0.0) (cl-tf:make-quaternion 1 0 -1 0)))
         (obj-rot (cl-tf:make-transform-stamped "base_footprint" "base_footprint" 0 (cl-tf:make-3d-vector 0.0 0 0.0)
                                        (cl-tf:orientation (btr:pose  (btr:object btr:*current-bullet-world* name))))))

    (cram-tf:apply-transform obj-rot std-transf)))

(defmethod calculate-pre-opening-trajectory ((object-type (eql :removed-by-hand)) init-pose name (tool (eql nil)) &optional (angle (cram-math:degrees->radians 10)))
  init-pose)

(defmethod calculate-opening-trajectory ((object-type (eql :removed-by-hand)) init-pose name (tool (eql nil)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 36)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (lift-offset (/ obj-height (* 2 times)))
         (result init-pose)
         (new-pose (cram-tf:apply-transform
                    (cram-tf:pose-stamped->transform-stamped (car (last result)) "base_footprint")
                    (cl-transforms-stamped:make-transform-stamped
                      "map" "base_footprint" 0
                      (cl-tf:make-3d-vector -0.1 0 0.0) (cl-tf:orientation (car (last result)))))))
    
    (dotimes (n times)
      (let* ((offset-transf (cram-tf:apply-transform
                             (cram-tf:pose-stamped->transform-stamped (car (last result)) "base_footprint")
                             (cl-transforms-stamped:make-transform-stamped
                              "map" "base_footprint" 0
                              (cl-tf:make-3d-vector (- lift-offset) 0 0.0) (cl-tf:orientation (car (last result))))))
             (opening-pose (cl-tf:make-pose-stamped "base_footprint" 0
                                                    (cl-tf:translation offset-transf)
                                                    (cl-tf:orientation (cram-tf:rotate-pose-in-own-frame (car (last result)) :x  (- angle))))))
                           ;;:z lift-offset)))
        (push opening-pose (cdr (last result)))))
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

(defmethod calculate-pre-opening-trajectory ((object-type (eql :removed-by-caplifter)) init-pose name (tool (eql :caplifter)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 8)
         (obj-radius (cl-transforms:x
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (shift-offset (+ (/ obj-radius (* 2 times))))
         (result (last init-pose)))
    (dotimes (n (/ times 1))
      (let ((opening-pose (cram-tf:translate-pose (car (last result)) :x (* 2 shift-offset))))
        (push opening-pose (cdr (last result)))))
    
    result))

(defmethod calculate-opening-trajectory ((object-type (eql :removed-by-caplifter)) init-pose name (tool (eql :caplifter)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 10)
         (obj-pose (btr:pose  (btr:object btr:*current-bullet-world* name)))
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         ;; (oTm (cram-tf:pose->transform-stamped
         ;;       "beerobottlecap_1"
         ;;       cram-tf:*fixed-frame*
         ;;       0.0
         ;;       obj-pose))
         (mTb (cram-tf:pose->transform-stamped
               cram-tf:*fixed-frame*
               cram-tf:*robot-base-frame*
               0.0
               (btr:pose (btr:get-robot-object))))

         (transf (lookup-transform-using-map
                  obj-pose
                  (cram-tf:apply-transform mTb (cram-tf:pose-stamped->transform-stamped (car init-pose) "beerbottlecap_1") :result-as-pose-or-transform :pose) "bottleopener_1" "beerbottlecap_1"))
         

         (new-pose (cl-tf:copy-pose-stamped
                    (car init-pose)
                    :origin (cl-tf:v+ (cl-tf:origin (car (last init-pose)))
                                                    ;;(car init-pose))
                                      (cl-tf:make-3d-vector 0
                                                            ;;0
                                                            0.005
                                                            (+ (/ obj-height 2)
                                                               (cl-tf:z (cl-tf:translation transf)))))
                    :orientation (cl-tf:make-quaternion 0 0.707 0 0.707)))) ;; 0.5 0.5 -0.5 0.5))))
    (interpolate-pose (car (last init-pose)) new-pose 10)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cork ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-open-reach-transform ((object-type (eql :removed-by-corkscrew)) name arm (grasp (eql :top)) (tool (eql :corkscrew)))

  (let* ((z (cl-transforms:z
             (cl-bullet::bounding-box-dimensions
              (btr::aabb  (btr:object btr:*current-bullet-world* name))))))
    
    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0
     (cl-tf:make-3d-vector 0.0 0 0.0) (cl-tf:make-quaternion 1 0 -1 0))))

(defmethod calculate-pre-opening-trajectory ((object-type (eql :removed-by-corkscrew)) init-pose name (tool (eql :corkscrew)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 36)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (lift-offset (- (/ obj-height (* 2 times))))
         (result init-pose))
    (dotimes (n times)
      (let ((opening-pose (cram-tf:translate-pose
                           (rotate-once-pose (car (last result)) (- angle) :z)
                           :z lift-offset)))

        (push opening-pose (cdr (last result)))))
    
    result))
     

(defmethod calculate-opening-trajectory ((object-type (eql :removed-by-corkscrew)) init-pose name (tool (eql :corkscrew)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 36)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (lift-offset (- (/ obj-height (* 2 times))))
         (result (last init-pose)))
    (dotimes (n (/ times 2))
      (let ((opening-pose (cram-tf:translate-pose (car (last result)) :z (- (* 4 lift-offset)))))
        (push opening-pose (cdr (last result)))))
    
    result))

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
