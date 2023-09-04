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

(in-package :demo)

(defun translate-pose-in-base (bTg &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (cram-tf:translate-transform-stamped bTg
                                       :x-offset x-offset
                                       :y-offset y-offset
                                       :z-offset z-offset))

(defun calculate-init-slicing-pose (object arm bTg)
  (let* ((x-gripper-position-offset
           (/(cl-transforms:y
              (cl-bullet::bounding-box-dimensions
               (btr:aabb object)))
             2))
         (y-gripper-position-offset
           (/(cl-transforms:x
              (cl-bullet::bounding-box-dimensions
               (btr:aabb object)))
             2)))
    (translate-pose-in-base 
     bTg
     :x-offset (- x-gripper-position-offset)
     :y-offset (if (eq arm :right)
                   (- y-gripper-position-offset)
                   y-gripper-position-offset))))

(defun calculate-slicing-trajectory-in-map (object arm bTg)
  (let* ((mTb
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            cram-tf:*robot-base-frame*
            0.0
            (btr:pose (btr:get-robot-object)))))
    (mapcar (lambda (bTg-pose)
               (cl-tf:ensure-pose-stamped
                (cram-tf:apply-transform 
                 mTb
                 bTg-pose)))
             (calculate-slicing-trajectory object arm bTg))))

(defun calculate-slicing-trajectory (object arm bTg
                                     &optional (slicing-thickness 0.02))

   (let* ((x-dim-object
            (cl-transforms:x
             (cl-bullet::bounding-box-dimensions
            (btr::aabb object))))
          (n-times-cut-value
            (round 
             (/ (* 2 x-dim-object)
                slicing-thickness)))
          (slice-poses
            `(,(calculate-init-slicing-pose object arm bTg))))

     (dotimes (n (- n-times-cut-value 3))
       (let ((slice-adjustment-pose
               (translate-pose-in-base (car (last slice-poses)) 
                                       :y-offset (if (eq arm :right)
                                                     slicing-thickness
                                                     (- slicing-thickness)))))
         (push slice-adjustment-pose
               (cdr (last slice-poses)))))
     slice-poses))



;;get sclicing trajectory has the poses:
;;reaching,graspinng,lifting,slice-up,slice down
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :slicing))
                                                          arm
                                                          grasp
                                                          location
                                                          objects-acted-on
                                                          &key )


  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (btr-object
           (btr:object btr:*current-bullet-world* object-name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-old-transform object))
         (oTb
           (cram-tf:transform-stamped-inv bTo))
         (bTb-lifts
           (man-int:get-object-type-wrt-base-frame-lift-transforms
            object-type arm grasp location))
         (oTg-std 
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (oTg-lifts
           (reverse
            (mapcar
             (lambda (btb-lift)
               (reduce #'cram-tf:apply-transform
                       `(,oTb ,bTb-lift ,bTo ,oTg-std)
                       :from-end T))
             bTb-lifts))))

    (flet ((get-base-to-gripper-transform-for-slicing (bTb-offset)
             (cl-tf:make-transform-stamped
              (cl-tf:frame-id bTo)
              (if (eql arm :right)
                  "r_gripper_tool_frame"
                  "l_gripper_tool_frame")
              0.0
              (cl-tf:v+
               (cl-tf:translation bTo)
               (cl-tf:translation bTb-offset))
              (cl-tf:rotation bTb-offset))))
      
      (mapcar (lambda (label transforms)
                (man-int:make-traj-segment
                 :label label
                 :poses 
                 (if (or (eq label :slice-up) 
                         (eq label :slice-down))
                     (calculate-slicing-trajectory-in-map btr-object arm
                                                          (get-base-to-gripper-transform-for-slicing 
                                                           (car transforms)))
                     (mapcar 
                      (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                      transforms))))
              '(:reaching
                :grasping
                :lifting
                :slice-up
                :slice-down)
              `(,(man-int:get-object-type-to-gripper-pregrasp-transforms
                  object-type object-name arm grasp location oTg-std)
                (,oTg-std)
                ,oTg-lifts
                (,(man-int:get-object-type-robot-frame-slice-up-transform
                   object-type arm grasp))
                (,(man-int:get-object-type-robot-frame-slice-down-transform
                   object-type arm grasp)))))))

(defun get-tilting-poses (grasp approach-poses &optional (angle (cram-math:degrees->radians 100)))
  (mapcar (lambda (?approach-pose)
            ;;depending on the grasp the angle to tilt is different
            (case grasp
              (:front (rotate-once-pose ?approach-pose (+ angle) :y))
              (:left-side (rotate-once-pose ?approach-pose (+ angle) :x))
              (:right-side (rotate-once-pose ?approach-pose (- angle) :x))
              (:back (rotate-once-pose ?approach-pose (- angle) :y))
              (t (error "can only pour from :side, back or :front"))))
          approach-poses))

;;helper function for tilting
;;rotate the pose around the axis in an angle
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

;;get pouring trajectory workes like picking-up it will get the 
;;object-type-to-gripper-tilt-approch-transform und makes a traj-segment out of it
;;here we have only the approach pose, followed by that is the titing pose (above)
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :pouring))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key )
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object))
         ;; The first part of the btb-offset transform encodes the
         ;; translation difference between the gripper and the
         ;; object. The static defined orientation of bTb-offset
         ;; describes how the gripper should be orientated to approach
         ;; the object in which something should be poured into. This
         ;; depends mostly on the defined coordinate frame of the
         ;; object and how objects should be rotated to pour something
         ;; out of them.
         (bTb-offset
           (man-int::get-object-type-robot-frame-tilt-approach-transform
            object-type arm grasp))
         ;; Since the grippers orientation should not depend on the
         ;; orientation of the object it is omitted here.
         (oTg-std
           (cram-tf:copy-transform-stamped
            (man-int:get-object-type-to-gripper-transform
             object-type object-name arm grasp)
            :rotation (cl-tf:make-identity-rotation)))
         (approach-pose
           (cl-tf:copy-pose-stamped 
            (man-int:calculate-gripper-pose-in-base
              (cram-tf:apply-transform
               (cram-tf:copy-transform-stamped 
                bTb-offset
                :rotation (cl-tf:make-identity-rotation))
               bTo)
              arm oTg-std)
            :orientation 
            (cl-tf:rotation bTb-offset)))
            ;; (cl-tf:q+
            ;;  (cl-tf:q-
            ;;   (cl-tf:orientation (btr:pose (btr:get-robot-object)))
            ;;   (cl-tf:rotation
            ;;    (cram-tf:apply-transform
            ;;     (cl-tf:make-transform-stamped
            ;;      cram-tf:*robot-base-frame*
            ;;      cram-tf:*robot-base-frame*
            ;;      0.0
            ;;      (cl-tf:make-identity-vector)
            ;;      (cl-tf:rotation (cl-tf:transform-inv bTo)))
            ;;     (cl-tf:make-transform-stamped
            ;;      cram-tf:*robot-base-frame*
            ;;      cram-tf:*robot-base-frame*
            ;;      0.0
            ;;      (cl-tf:make-identity-vector)
            ;;      (cl-tf:orientation (btr:pose
            ;;                          (btr:get-robot-object)))))))
            ;;  (case grasp
            ;;    (:FRONT (cl-tf:euler->quaternion :az 0.0))
            ;;    (:BACK (cl-tf:euler->quaternion :az pi))
            ;;    (:LEFT-SIDE (cl-tf:euler->quaternion :az (- (/ pi 2))))
            ;;    (:RIGHT-SIDE (cl-tf:euler->quaternion :az (/ pi 2)))))))
         (tilting-poses
           (get-tilting-poses grasp (list approach-pose))))
    (mapcar (lambda (label poses-in-base)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar 
                       (lambda (pose-in-base)
                         (let ((mTb (cram-tf:pose->transform-stamped
                                     cram-tf:*fixed-frame*
                                     cram-tf:*robot-base-frame*
                                     0.0
                                     (btr:pose (btr:get-robot-object))))
                               (bTg-std
                                 (cram-tf:pose-stamped->transform-stamped
                                  pose-in-base
                                  (cl-tf:child-frame-id bTo))))
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg-std))))
                       poses-in-base)))
            '(:approach
              :tilting)
            `((,approach-pose)
              ,tilting-poses))))


(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :removing-cap))
                                                 arm
                                                 grasp
                                                 location
                                                 objects-acted-on
                                                 &key)
  (let* ((object
           (car objects-acted-on))
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
         (bTb-lifts
           (man-int:get-object-type-wrt-base-frame-lift-transforms
            object-type arm grasp location))
         (oTg-lifts
           (mapcar (lambda (btb-lift)
                     (reduce #'cram-tf:apply-transform
                             `(,oTb ,bTb-lift ,bTo ,oTg-std)
                             :from-end T))
                   bTb-lifts))
         (oTg-pregrasps
           (man-int:get-object-type-to-gripper-pregrasp-transforms
            object-type object-name arm grasp location oTg-std)))

    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                              transforms)))
            '(:reaching
              :grasping
              :lifting)
            `(,oTg-pregrasps
              (,oTg-std)
              ,oTg-lifts))))


(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :removing-cap2))
                                                 arm
                                                 grasp
                                                 location
                                                 objects-acted-on
                                                 &key)
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            ;; object-type
            :cap
            object-name arm grasp))
         (bTo
           (man-int:get-object-transform object))
         (oTb
           (cram-tf:transform-stamped-inv bTo))
         (bTb-lifts
           (man-int:get-object-type-wrt-base-frame-lift-transforms
            ;; object-type
            :cap
            arm grasp location))
         (oTg-lifts
           (mapcar (lambda (btb-lift)
                     (reduce #'cram-tf:apply-transform
                             `(,oTb ,bTb-lift ,bTo ,oTg-std)
                             :from-end T))
                   bTb-lifts))
         (oTg-pregrasps
           (man-int:get-object-type-to-gripper-pregrasp-transforms
            ;; object-type
            :cap
            object-name arm grasp location oTg-std)))

    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                              transforms)))
            '(:reaching
              :grasping
              :lifting)
            `(,oTg-pregrasps
              (,oTg-std)
              ,oTg-lifts))))

;;get pouring trajectory workes like picking-up it will get the 
;;object-type-to-gripper-tilt-approch-transform und makes a traj-segment out of it
;;here we have only the approach pose, followed by that is the titing pose (above)
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :screwing-open))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key )
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object))
         ;; The first part of the btb-offset transform encodes the
         ;; translation difference between the gripper and the
         ;; object. The static defined orientation of bTb-offset
         ;; describes how the gripper should be orientated to approach
         ;; the object in which something should be poured into. This
         ;; depends mostly on the defined coordinate frame of the
         ;; object and how objects should be rotated to pour something
         ;; out of them.
         (bTb-offset
           (get-object-type-robot-frame-open-bottle-approach-transform
            object object-type arm grasp))
         ;; Since the grippers orientation should not depend on the
         ;; orientation of the object it is omitted here.
         (oTg-std
           (cram-tf:copy-transform-stamped
            (man-int:get-object-type-to-gripper-transform
             object-type object-name arm grasp)
            :rotation (cl-tf:make-identity-rotation)))
         (approach-pose
           (cl-tf:copy-pose-stamped 
            (man-int:calculate-gripper-pose-in-base
              (cram-tf:apply-transform
               (cram-tf:copy-transform-stamped 
                bTb-offset
                :rotation (cl-tf:make-identity-rotation))
               bTo)
              arm oTg-std)
            :orientation 
            (cl-tf:rotation bTb-offset)))
          
         (opening-poses
           (calculate-opening-trajectory grasp (list approach-pose))))
    (mapcar (lambda (label poses-in-base)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar 
                       (lambda (pose-in-base)
                         (let ((mTb (cram-tf:pose->transform-stamped
                                     cram-tf:*fixed-frame*
                                     cram-tf:*robot-base-frame*
                                     0.0
                                     (btr:pose (btr:get-robot-object))))
                               (bTg-std
                                 (cram-tf:pose-stamped->transform-stamped
                                  pose-in-base
                                  (cl-tf:child-frame-id bTo))))
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg-std))))
                       poses-in-base)))
            '(:approach
              :tilting)
            `((,approach-pose)
              ,opening-poses))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :open-bottle))
                                                         (arm (eql :left))
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key tilt-angle side)

  (print "this is in pouring trajectories")
  (let* ((side :top)
         (target-object
           objects-acted-on)
         (target-object-name
           (desig:desig-prop-value target-object :name))
         (target-object-type
           (desig:desig-prop-value target-object :type))
         (b-T-to
           (man-int:get-object-transform target-object))

         ;;;;vvvvvvvvv;;;;;;
         (to-T-to-offset
           (get-object-type-robot-frame-open-approach-transform
            target-object-name target-object-type arm side))
         
         ;; Since the grippers orientation should not depend on the
         ;; orientation of the object it is omitted here.
         (oTg-std
           (cram-tf:copy-transform-stamped
            (man-int:get-object-type-to-gripper-transform
             target-object-type target-object-name arm side)
            :rotation (cl-tf:make-identity-rotation)))

         (approach-pose
           (cl-tf:copy-pose-stamped 
            (man-int:calculate-gripper-pose-in-base
              (cram-tf:apply-transform
               (cram-tf:copy-transform-stamped 
                to-T-to-offset
                :rotation (cl-tf:make-identity-rotation))
               b-T-to)
              arm oTg-std)
            :orientation 
            (cl-tf:rotation to-T-to-offset)))
         (pre-opening-poses
           (calculate-pre-opening-trajectory (list approach-pose) target-object-name target-object-type))
         (opening-poses
           (calculate-opening-trajectory (copy-list pre-opening-poses) target-object-name target-object-type)))
    
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
                                  (cl-tf:child-frame-id b-T-to))))
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg-std))))
                       poses-in-base)))
            
            `(:approach
              :grasping
              :pre-open
              :open)
            `((,approach-pose)
              (,approach-pose)
              ,pre-opening-poses
              ,opening-poses))))


(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :open-bottle))
                                                 (arm (eql :right))
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
            '(:approach
              :grasping)
            `(,oTg-pregrasps
              (,oTg-std)))))


(defmethod get-object-type-robot-frame-open-approach-transform (name (object-type (eql :milk)) (arm (eql :right)) (grasp (eql :top)))

  (let* ((z (cl-transforms:z
             (cl-bullet::bounding-box-dimensions
              (btr::aabb  (btr:object btr:*current-bullet-world* name))))))
    
    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0
     (cl-tf:make-3d-vector  0 0 0.00
                           ;; 0 0 (/ z 2)
                            )
     (cl-tf:make-quaternion 1 0 -1 0))))


(defmethod get-object-type-robot-frame-open-approach-transform (name (object-type (eql :milkbottlecap)) (arm (eql :left)) (grasp (eql :top)))

  (let* ((z (cl-transforms:z
             (cl-bullet::bounding-box-dimensions
              (btr::aabb  (btr:object btr:*current-bullet-world* name))))))
    
    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0
     (cl-tf:make-3d-vector 0.0 0 0.0) (cl-tf:make-quaternion 1 0 -1 0))));;??

(defmethod get-object-type-robot-frame-open-approach-transform (name (object-type (eql :beerbottlecap)) (arm (eql :left)) (grasp (eql :top)))

  (let* ((rot (cl-tf:make-quaternion  0.37 0.37 -0.603 0.603))
         (obj-radius (cl-transforms:x
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name))))))

    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0
     (cl-tf:make-3d-vector 0 (+ 0.09 obj-radius) -0.135) rot)))

(defmethod get-object-type-robot-frame-open-approach-transform (name (object-type (eql :cork)) (arm (eql :left)) (grasp (eql :top)))

  (let* ((z (cl-transforms:z
             (cl-bullet::bounding-box-dimensions
              (btr::aabb  (btr:object btr:*current-bullet-world* name))))))
    
    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0
     (cl-tf:make-3d-vector 0.0 0 0.0) (cl-tf:make-quaternion 1 0 -1 0)))) 

;; (defun calculate-init-opening-pose (object arm bTg)
;;   (let* ((z-gripper-position-offset
;;            (/(cl-transforms:z
;;               (cl-bullet::bounding-box-dimensions
;;                (btr:aabb object)))
;;              2)))
;;     (translate-pose-in-base 
;;      bTg
;;      :z-offset z-gripper-position-offset)))


;; (defun calculate-opening-trajectory (init-pose &optional (angle (cram-math:degrees->radians 50)))
;;   (let* ((times (ceiling (* 2 (/ (* 2 pi) angle))))
;;          (pose (first init-pose))
;;          (result init-pose))
;;     (dotimes (n times)
;;       (let ((opening-pose
;;               (rotate-once-pose pose (- angle) :z)))
;;         (setf pose opening-pose)
;;         (push opening-pose
;;               (cdr (last result)))))
;;     result))

(defmethod calculate-opening-trajectory (init-pose name (object-type (eql :milkbottlecap)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 36)
         (obj-height (cl-transforms:z
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (lift-offset (/ obj-height (* 2 times)))
         (result init-pose))
    (dotimes (n times)
      (let ((opening-pose (cram-tf:translate-pose
                           (rotate-once-pose (car (last result)) angle :z)
                           :z lift-offset)))
        (push opening-pose (cdr (last result)))))
    result))

(defmethod calculate-opening-trajectory (init-pose name (object-type (eql :beerbottlecap)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 10)
         (obj-pose (btr:pose  (btr:object btr:*current-bullet-world* :beerbottlecap-1)))
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
         
         ;; (transl-pose (cram-tf:translate-pose (car init-pose)
         ;;                                      :x (cl-tf:x (cl-tf:translation transf))
         ;;                                      :y (cl-tf:y (cl-tf:translation transf))
         ;;                                      :z (cl-tf:z (cl-tf:translation transf))))

         ;; (rot-pose (cram-tf:rotate-transform-in-own-frame transf :y 0.01))
         
         ;; (applied-transf rot-pose)
           ;; (cram-tf:apply-transform rot-pose
           ;;                          transf))

         (new-pose (cl-tf:copy-pose-stamped
                    (car init-pose)
                    :origin (cl-tf:v+ (cl-tf:origin (car (last init-pose)))
                                                    ;;(car init-pose))
                                      (cl-tf:make-3d-vector 0
                                                            ;;0
                                                            0.005
                                                            (+ (/ obj-height 2)
                                                               (cl-tf:z (cl-tf:translation transf)))))
                    :orientation (cl-tf:make-quaternion 0.5 0.5 -0.5 0.5))))
    (interpolate-pose (car (last init-pose)) new-pose 10)
    ;; (dotimes (n times)
    ;;   (let ((opening-pose (cram-tf:translate-pose
    ;;                        (rotate-once-pose (car (last result)) angle :z)
    ;;                        :z lift-offset)))
    ;;     (push opening-pose (cdr (last result)))))
   ))

(defmethod calculate-pre-opening-trajectory (init-pose name (object-type (eql :cork)) &optional (angle (cram-math:degrees->radians 10)))
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
     

(defmethod calculate-opening-trajectory (init-pose name (object-type (eql :cork)) &optional (angle (cram-math:degrees->radians 10)))
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

(defmethod calculate-pre-opening-trajectory (init-pose name (object-type (eql :beerbottlecap)) &optional (angle (cram-math:degrees->radians 10)))
  (let* ((times 8)
         (obj-radius (cl-transforms:x
                      (cl-bullet::bounding-box-dimensions
                       (btr::aabb  (btr:object btr:*current-bullet-world* name)))))
         (shift-offset (+ (/ obj-radius (* 2 times))))
         (result (last init-pose)))
    (dotimes (n (/ times 1))
      (let ((opening-pose (cram-tf:translate-pose (car (last result)) :y (- (* 2 shift-offset)))))
        (push opening-pose (cdr (last result)))))
    
    result))

(defmethod calculate-pre-opening-trajectory (init-pose name (object-type t) &optional (angle (cram-math:degrees->radians 10)))
  init-pose)


(defun rotate-coords (p rot)
  (let* ((rot-conj (cl-tf:make-quaternion (- (cl-tf:x rot)) (- (cl-tf:y rot)) (- (cl-tf:z rot)) (cl-tf:w rot)))
         (p-as-quaternion (cl-tf:make-quaternion (cl-tf:x p) (cl-tf:y p) (cl-tf:z p) 0))
         (rotated-p (cl-tf:q* rot p-as-quaternion rot-conj))
         new-coordinates)
    
    ;; Extract the new coordinates of point B from the rotated quaternion
    (setf new-coordinates(cl-tf:make-3d-vector (cl-tf:x rotated-p) (cl-tf:y rotated-p) (cl-tf:z rotated-p)))
    
    ;; Print the new coordinates of point B
    (format t "New coordinates of point B: ~A" new-coordinates)
    new-coordinates))

(defun calc-rotation ()
  (let* ((pos1 (cl-tf:make-3d-vector
                0.7626845194658584d0
                -0.017200075319338353d0
                0.9853611683366628d0))
         (rot1 (cl-tf:quaternion->matrix (cl-tf:make-quaternion
                0.3698118237163824
                0.3698118237163824
                -0.6026933009745199
                0.6026933009745199)))
         (pos2 (cl-tf:make-3d-vector
                0.7626845194658584d0
                -0.057200074425268685d0
                1.0333267173767091d0))
         (rot2 (cl-tf:quaternion->matrix (cl-tf:make-quaternion 0.5 0.5 -0.5 0.5)))
         (pos3 (cl-tf:make-3d-vector
                1.4626845041910808d0
                0.5560557683308919d0
                0.9989501317342122d0))
         (rot3 (cl-tf:quaternion->matrix (cl-tf:make-quaternion
                -0.16467225551605225
                -0.16467225551605225
                -0.6876649260520935
                0.6876649260520935)))
         (rel-rot (dot-product rot3 (cl-tf:invert-rot-matrix rot2)))
         (rel-transl (dot-product rel-rot
                                  (make-array '(3 1) :initial-contents `((,(cl-tf:x pos1))
                                                                         (,(cl-tf:y pos1))
                                                                         (,(cl-tf:z pos1))))))
         (pose-rot (dot-product rel-rot rot1)))
    (cl-tf:make-pose-stamped
     "map" 0
     (cl-tf:v+ (cl-tf:make-3d-vector (aref rel-transl 0 0)
                                     (aref rel-transl 1 0)
                                     (aref rel-transl 2 0))
               pos3)
     (cl-tf:matrix->quaternion pose-rot))
    (print rel-rot)
    (print rel-transl)
    ;; (cl-tf:normalize (cl-tf:matrix->quaternion pose-rot))
    ))

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

;; (defun slerp (q1 q2 t-val)
;;   (let ((dot-prod (cl-tf:q-dot q2 q1))
;;         result
;;         theta
;;         sin-theta
;;         s0
;;         s1)
;;     (when (< dot-prod 0.0)
;;       (setf q1 (cl-tf:q-inv q1))
;;       (setf dot-prod (- dot-prod)))
    
;;     (when (> dot-prod 0.9999)
;;       (setf result (cl-tf:q+ q1 (cl-tf:q* t-val (cl-tf:q- q2 q1)))))
    
;;     (setf theta (* t-val (acos dot-prod)))
;;     (setf sin-theta (sin theta))
;;     (setf s0 (- (cos theta) (* dot-prod sin-theta)))
;;     (setf s1 sin-theta)

;;     (cl-tf:q+ (cl-tf:q* s0 q1) (cl-tf:q* s1 q2))))

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

(;; defun interpolate-rotation (start-pose end-pose steps)
 ;;  (let ((i steps)
 ;;        interp)
 ;;    (loop while (> i 0)
 ;;          do
 ;;             (let* ((t-val (/ i (- steps 1)))
 ;;                    (q (slerp start-q end-q t-val)))
 ;;               (push q interp)
 ;;               (format t "Step: ~a, interpolated quaternion: ~a ~%" i q))
 ;;             (setf i (- i 1)))))

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


     ;; (start-q (cl-tf:make-quaternion
     ;;              0.3698118237163824d0
     ;;              0.3698118237163824d0
     ;;              -0.6026933009745199d0
     ;;              0.6026933009745199d0))
     ;;    (end-q (cl-tf:make-quaternion 0.5 0.5 -0.5 0.5))
