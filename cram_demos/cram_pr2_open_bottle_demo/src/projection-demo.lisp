;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demo)

(defparameter *object-spawning-poses*
  '("sink_area_surface"
    ((:breakfast-cereal . ((0.2 -0.15 0.1) (0 0 0 1)))
     (:cup . ((0.2 -0.35 0.1) (0 0 0 1)))
     (:bowl . ((0.18 -0.55 0.15) (0 0 0 1)))
     (:spoon . ((0.15 -0.4 -0.05) (0 0 0 1)))
     (:milk . ((0.17 -0.35 0.1) (0 0 0 1)))
     (:milkbottle . ((0.17 -0.15 0.1) (0 0 0 1)))
     (:milkpack . ((0.17 -0.15 0.1) (0 0 0 1)))
     (:winebottle . ((0.17 -0.15 0.1) (0 0 0 1)))
     (:corkscrew . ((0.17 -0.35 0.1) (0 0 0 1)))
     (:electric-corkscrew . ((0.17 -0.35 0.1) (0 0 1 0)))
     (:caplifter . ((0.17 -0.35 0.1) (0 0 0 1)))
     (:beerbottle . ((0.17 -0.15 0.1) (0 0 0 1)))
     (:beerbottle-tall . ((0.17 -0.15 0.1) (0 0 0 1)))
     (:albihimbeerjuice . ((0.17 -0.15 0.1) (0 0 0 1)))))
  "Relative poses on sink area")


(defparameter *object-placing-poses*
  '((:breakfast-cereal . ((-0.78 0.9 0.95) (0 0 1 0)))
    (:cup . ((-0.79 1.35 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.76 1.19 0.88) (0 0 0.7071 0.7071)))
    (:spoon . ((-0.78 1.5 0.86) (0 0 0 1)))
    (:milk . ((-0.75 1.7 0.95) (0 0 0.7071 0.7071))))
  "Absolute poses on kitchen_island.")

(defparameter *object-grasping-arms*
  '(;; (:breakfast-cereal . :right)
    ;; (:cup . :left)
    ;; (:bowl . :right)
    ;; (:spoon . :right)
    ;; (:milk . :right)
    ))


(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))
(defparameter *island-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))


(defun make-poses-relative (spawning-poses)
  "Gets an associative list in a form of (FRAME ((TYPE . COORDINATES-LIST) ...)),
where coordinates-list is defined in the FRAME coordinate frame.
Converts these coordinates into CRAM-TF:*FIXED-FRAME* frame and returns a list in form
 ((TYPE . POSE) ...)."
  (when spawning-poses
    (let* ((map-T-surface (cl-transforms:pose->transform
                           (btr:link-pose (btr:get-environment-object)
                                          (first spawning-poses)))))
      (mapcar (lambda (type-and-pose-list)
                (destructuring-bind (type . pose-list)
                    type-and-pose-list
                  (let* ((surface-T-object
                           (cl-transforms:pose->transform (cram-tf:list->pose pose-list)))
                         (map-T-object
                           (cl-transforms:transform* map-T-surface surface-T-object))
                         (map-P-object
                           (cl-transforms:transform->pose map-T-object)))
                    `(,type . ,map-P-object))))
              (second spawning-poses)))))


(defun spawn-objects-on-sink-counter (&key
                                        (object-types '(;; :breakfast-cereal
                                                        ;; :cup
                                                        ;; :bowl
                                                        ;; :spoon
                                                        ;; :milk
                                                        :milkpack
                                                        ))
                                        (spawning-poses-relative *object-spawning-poses*)
                                        (random NIL))
  ;; make sure mesh paths are known, kill old objects and destroy all attachments
  (btr:add-objects-to-mesh-list "cram_pr2_open_bottle_demo")
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))

  ;; spawn objects
  (let* ((spawning-poses-absolute
           (make-poses-relative spawning-poses-relative))
         (objects
           (mapcar (lambda (object-type)
                     (let* (;; generate object name of form :TYPE-1
                            (object-name
                              (intern (format nil "~a-1" object-type) :keyword))
                            ;; spawn object in Bullet World at default pose
                            (object
                              (btr-utils:spawn-object object-name object-type))
                            ;; calculate new pose: either random or from input list
                            (object-pose
                              (if random
                                  ;; generate a pose on a surface
                                  (let* ((aabb-z
                                           (cl-transforms:z
                                            (cl-bullet:bounding-box-dimensions
                                             (btr:aabb object)))))
                                    (cram-tf:translate-pose
                                     (desig:reference
                                      (if (eq object-type :spoon)
                                          (desig:a location
                                                   (in (desig:an object
                                                                 (type drawer)
                                                                 (urdf-name
                                                                  sink-area-left-upper-drawer-main)
                                                                 (part-of iai-kitchen)))
                                                   (side front)
                                                   (range 0.2)
                                                   (range-invert 0.12))
                                          (desig:a location
                                                   (on (desig:an object
                                                                 (type counter-top)
                                                                 (urdf-name sink-area-surface)
                                                                 (part-of iai-kitchen)))
                                                   ;; below only works for knowrob sem-map
                                                   ;; (centered-with-padding 0.1)
                                                   (side left)
                                                   (side front))))
                                     :z (/ aabb-z 2.0)))
                                  ;; take the pose from the function input list
                                  (cdr (assoc object-type spawning-poses-absolute))))
                            ;; rotate new pose randomly around Z
                            (rotated-object-pose
                              (cl-tf:copy-pose object-pose
                                               :orientation (cl-tf:make-quaternion 0 0 1 0))))
                              ;; (cram-tf:rotate-pose object-pose
                              ;;                      :z (/ (* 2 pi) (random 10.0)))))

                       ;; move object to calculated pose on surface
                       (btr-utils:move-object object-name rotated-object-pose)
                       ;; return object
                       object))
                   object-types)))

    ;; make sure generated poses are stable, especially important for random ones
    ;; TDOO: if unstable, call itself
    
    
     ;; attach spoon to the drawer
    (when (btr:object btr:*current-bullet-world* :spoon-1)
      (btr:attach-object (btr:get-environment-object)
                         (btr:object btr:*current-bullet-world* :spoon-1)
                         :link "sink_area_left_upper_drawer_main"))

    ;; stabilize world
    (btr:simulate btr:*current-bullet-world* 100)

    (when (btr:object btr:*current-bullet-world* :milkbottle-1)
      (let* ((milk-map-transf (cram-tf:pose->transform-stamped "map" "milkbottle_1" 0  (btr:object-pose :milkbottle-1)))
             (milk-cap-transf (cl-tf:make-transform-stamped "milkbottle_1" "milkbottle_1" 0 (cl-tf:make-3d-vector 0 -0.0025 0.095) (cl-tf:make-identity-rotation)))
             (cap-pose (cram-tf:apply-transform milk-map-transf milk-cap-transf :result-as-pose-or-transform :pose)))
        (btr-utils:spawn-object :milkbottlecap-1 :milkbottlecap
                                :pose cap-pose)            
        (btr:attach-object (btr:object btr:*current-bullet-world* :milkbottle-1)
                           (btr:object btr:*current-bullet-world* :milkbottlecap-1)
                           :attachment-type :cap
                           :loose t)))

    (when (btr:object btr:*current-bullet-world* :milkpack-1)
      (let* ((milk-map-transf (cram-tf:pose->transform-stamped "map" "milkpack_1" 0  (btr:object-pose :milkpack-1)))
             (milk-cap-transf (cl-tf:make-transform-stamped "milkpack_1" "milkpack_1" 0 (cl-tf:make-3d-vector 0.022 0.00 0.068) (cl-tf:make-quaternion 0 0.383 0 0.924)))
             (cap-pose (cram-tf:apply-transform milk-map-transf milk-cap-transf :result-as-pose-or-transform :pose)))

        (btr-utils:spawn-object :milkpackcap-1 :milkpackcap
                                :pose cap-pose)            
        (btr:attach-object (btr:object btr:*current-bullet-world* :milkpack-1)
                           (btr:object btr:*current-bullet-world* :milkpackcap-1)
                           :attachment-type :cap
                           :loose t)))

    (when (btr:object btr:*current-bullet-world* :winebottle-1)
      
      (let* ((wine-map-transf (cram-tf:pose->transform-stamped "map" "winebottle_1" 0  (btr:object-pose :winebottle-1)))
             (wine-cap-transf (cl-tf:make-transform-stamped "winebottle_1" "winebottle_1" 0 (cl-tf:make-3d-vector 0 0 0.097) (cl-tf:make-identity-rotation))) ;; 0.14 can be perceived, 0.097 cork is just sticking out
             (cork-pose (cram-tf:apply-transform wine-map-transf wine-cap-transf :result-as-pose-or-transform :pose)))
        (btr-utils:spawn-object :cork-1 :cork
                                :pose cork-pose)
        ;;(break)
        (btr:attach-object (btr:object btr:*current-bullet-world* :winebottle-1)
                           (btr:object btr:*current-bullet-world* :cork-1)
                           :attachment-type :cap
                           :loose t)))

    (when (btr:object btr:*current-bullet-world* :beerbottle-1)
      
      (let* ((beer-map-transf (cram-tf:pose->transform-stamped "map" "beerbottle_1" 0  (btr:object-pose :beerbottle-1)))
             (beer-cap-transf (cl-tf:make-transform-stamped "beerbottle_1" "beerbottle_1" 0 (cl-tf:make-3d-vector 0 0 0.084) (cl-tf:make-identity-rotation)))
             (beer-pose (cram-tf:apply-transform beer-map-transf beer-cap-transf :result-as-pose-or-transform :pose)))
        (btr-utils:spawn-object :beerbottlecap-1 :beerbottlecap
                                :pose beer-pose)
        (btr:attach-object (btr:object btr:*current-bullet-world* :beerbottle-1)
                           (btr:object btr:*current-bullet-world* :beerbottlecap-1)
                           :attachment-type :cap
                           :loose t)))

    (when (btr:object btr:*current-bullet-world* :beerbottle-tall-1)
      
      (let* ((beer-map-transf (cram-tf:pose->transform-stamped "map" "beerbottle_tall_1" 0  (btr:object-pose :beerbottle-tall-1)))
             (beer-cap-transf (cl-tf:make-transform-stamped "beerbottle_tall_1" "beerbottle_tall_1" 0 (cl-tf:make-3d-vector 0 0 0.107) (cl-tf:make-identity-rotation)))
             (beer-pose (cram-tf:apply-transform beer-map-transf beer-cap-transf :result-as-pose-or-transform :pose)))
        (btr-utils:spawn-object :beerbottlecap-tall-1 :beerbottlecap-tall
                                :pose beer-pose)
        (btr:attach-object (btr:object btr:*current-bullet-world* :beerbottle-tall-1)
                           (btr:object btr:*current-bullet-world* :beerbottlecap-tall-1)
                           :attachment-type :cap
                           :loose t)))

     (when (btr:object btr:*current-bullet-world* :albihimbeerjuice-1)
      

       (let* ((albi-map-transf (cram-tf:pose->transform-stamped "map" "albihimbeerjuice_1" 0  (btr:object-pose :albihimbeerjuice-1)))
             (albi-cap-transf (cl-tf:make-transform-stamped "albihimbeerjuice_1" "albihimbeerjuice_1" 0 (cl-tf:make-3d-vector -0.015 -0.015 0.122) (cl-tf:make-identity-rotation)))
             (albi-pose (cram-tf:apply-transform albi-map-transf albi-cap-transf :result-as-pose-or-transform :pose)))
        (btr-utils:spawn-object :albihimbeerjuicecap-1 :albihimbeerjuicecap
                                :pose albi-pose)
        (btr:attach-object (btr:object btr:*current-bullet-world* :albihimbeerjuice-1)
                           (btr:object btr:*current-bullet-world* :albihimbeerjuicecap-1)
                           :attachment-type :cap
                           :loose t)))
    
    ;; return list of BTR objects
    objects))



(defun go-to-sink-or-island (&optional (sink-or-island :sink))
  (let ((?navigation-goal (ecase sink-or-island
                            (:sink *sink-nav-goal*)
                            (:island *island-nav-goal*)))
        (?ptu-goal *look-goal*))
    (cpl:par
      (exe:perform (desig:an action
                             (type parking-arms)))
      (exe:perform (desig:a motion
                            (type going)
                            (pose ?navigation-goal))))
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?ptu-goal)))))

(defun pick-object (&optional (?object-type :breakfast-cereal) (?arm :right))
  (go-to-sink-or-island :sink)
  (let* ((?object-desig
           (desig:an object (type ?object-type)))
         (?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?object-desig)))))
    (cpl:par
      (exe:perform (desig:an action
                             (type looking)
                             (object ?perceived-object-desig)))
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm ?arm)
                             (object ?perceived-object-desig))))))

(defun place-object (?target-pose &optional (?arm :right))
  (go-to-sink-or-island :island)
  (cpl:par
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?target-pose)))
    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)
                           (target (desig:a location
                                            (pose ?target-pose)))))))

(defun demo-hard-coded ()
  (spawn-objects-on-sink-counter)

  (urdf-proj:with-simulated-robot

    (dolist (object-type '(:breakfast-cereal :cup :bowl :spoon :milk))

      (let ((placing-target
              (cl-transforms-stamped:pose->pose-stamped
               "map" 0.0
               (cram-bullet-reasoning:ensure-pose
                (cdr (assoc object-type *object-placing-poses*)))))
            (arm-to-use
              (cdr (assoc object-type *object-grasping-arms*))))

        (pick-object object-type (or arm-to-use :left))
        (place-object placing-target (or arm-to-use :left))))))









;; (defun test-projection ()
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (exe:perform
;;        (let ((?pose (cl-transforms-stamped:make-pose-stamped
;;                      cram-tf:*robot-base-frame* 0.0
;;                      (cl-transforms:make-3d-vector -0.5 0 0)
;;                      (cl-transforms:make-identity-rotation))))
;;          (desig:a motion (type going) (pose ?pose))))
;;       (exe:perform
;;        (desig:a motion (type moving-torso) (joint-angle 0.3)))
;;       (exe:perform
;;        (desig:a motion (type opening-gripper) (gripper left)))
;;       (exe:perform
;;        (desig:a motion (type looking) (direction forward)))
;;       (exe:perform
;;        (let ((?pose (cl-transforms-stamped:make-pose-stamped
;;                      cram-tf:*robot-base-frame* 0.0
;;                      (cl-transforms:make-3d-vector 0.7 0.3 0.85)
;;                      (cl-transforms:make-identity-rotation))))
;;          (desig:a motion (type moving-tcp) (left-pose ?pose)))))))

;; (defun test-desigs ()
;;   (let ((?pose (desig:reference (desig:a location
;;                                          (on "CounterTop")
;;                                          (name "iai_kitchen_meal_table_counter_top")))))
;;     (desig:reference (desig:a location
;;                               (to see)
;;                               (object (desig:an object (at (desig:a location (pose ?pose)))))))))

;; (defun spawn-bottle ()
;;   (add-objects-to-mesh-list)
;;   (btr-utils:kill-all-objects)
;;   (btr-utils:spawn-object :bottle-1 :bottle :color '(1 0.5 0))
;;   (btr-utils:move-object :bottle-1 (cl-transforms:make-pose
;;                                     (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
;;                                     (cl-transforms:make-identity-rotation)))
;;   ;; stabilize world
;;   (btr:simulate btr:*current-bullet-world* 100))

;; (defun spawn-objects ()
;;   (let ((object-types (add-objects-to-mesh-list)))
;;     ;; spawn at default location
;;     (let ((objects (mapcar (lambda (object-type)
;;                              (btr-utils:spawn-object
;;                               (intern (format nil "~a-1" object-type) :keyword)
;;                               object-type))
;;                            object-types)))
;;       ;; move on top of counter tops
;;       (mapcar (lambda (btr-object)
;;                 (let* ((aabb-z (cl-transforms:z
;;                                 (cl-bullet:bounding-box-dimensions (btr:aabb btr-object))))
;;                        (new-pose (cram-tf:translate-pose
;;                                   (desig:reference
;;                                    (desig:a location
;;                                             (on "CounterTop")
;;                                             (name "iai_kitchen_meal_table_counter_top")))
;;                                   :z (/ aabb-z 2.0))))
;;                   (btr-utils:move-object (btr:name btr-object) new-pose)))
;;               objects)
;;       ;; bottle gets special treatment
;;       (btr-utils:move-object :bottle-1 (cl-transforms:make-pose
;;                                         (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
;;                                         (cl-transforms:make-identity-rotation)))))
;;   ;; stabilize world
;;   (btr:simulate btr:*current-bullet-world* 100))

;; (defparameter *meal-table-left-base-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "map"
;;    0.0
;;    (cl-transforms:make-3d-vector -1.12d0 -0.42d0 0.0)
;;    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
;; (defparameter *meal-table-right-base-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "map"
;;    0.0
;;    (cl-transforms:make-3d-vector -2.0547d0 -0.481d0 0.0d0)
;;    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
;; (defparameter *meal-table-left-base-look-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.75d0 -0.12d0 1.11d0)
;;    (cl-transforms:make-identity-rotation)))
;; (defparameter *meal-table-right-base-look-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
;;    (cl-transforms:make-identity-rotation)))
;; (defparameter *meal-table-left-base-look-down-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.7d0 -0.12d0 0.7578d0)
;;    (cl-transforms:make-identity-rotation)))


;; (defun prepare ()
;;   (cpl:with-failure-handling
;;           ((common-fail:low-level-failure (e)
;;              (roslisp:ros-warn (demo step-0) "~a" e)
;;              (return)))

;;         (let ((?navigation-goal *meal-table-right-base-pose*)
;;               (?ptu-goal *meal-table-right-base-look-pose*))
;;           (cpl:par
;; (exe:perform
;;  (desig:an action
;;            (type parking-arms)))
;;             (exe:perform (desig:a motion
;;                                   (type going)
;;                                   (pose ?navigation-goal))))
;;           (exe:perform (desig:a motion
;;                                 (type looking)
;;                                 (pose ?ptu-goal))))))
;; (defun test-pr2-plans ()
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (prepare))))

;; (defun test-projection-perception ()
;;   (spawn-objects)
;;   (test-pr2-plans)
;;   (cpl:sleep 1)
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (exe:perform
;;        (let ((?object-designator
;;                (desig:an object (type bottle))))
;;          (desig:a motion
;;                   (type detecting)
;;                   (object ?object-designator)))))))

;; (defun test-grasp-and-place-object (&optional (?object-type :bottle) (?arm :right))
;;   (let ((proj-result
;;           (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;             (cpl:top-level
;;               (prepare))
;;             (cpl:top-level
;;               (let ((?bottle-desig (desig:an object (type ?object-type))))
;;                 (flet ((step-1-inner ()
;;                          (let ((?perceived-bottle-desig (pp-plans::perceive ?bottle-desig)))
;;                            (cpl:par
;;                              (exe:perform (desig:an action
;;                                                     (type looking)
;;                                                     (object ?perceived-bottle-desig)))
;;                              (exe:perform (desig:an action
;;                                                     (type picking-up)
;;                                                     (arm ?arm)
;;                                                     (object ?perceived-bottle-desig)))))))
;;                   (cpl:with-retry-counters ((bottle-grasp-tries 2))
;;                     (cpl:with-failure-handling
;;                         ((common-fail:low-level-failure (e)
;;                            (roslisp:ros-warn (demo step-1) "~a" e)
;;                            (cpl:do-retry bottle-grasp-tries
;;                              (roslisp:ros-warn (demo step-1) "~a" e)
;;                              (prepare)
;;                              (cpl:retry))))

;;                       (step-1-inner))))
;;                 (desig:current-desig ?bottle-desig))))))
;;     (cpl:sleep 1.0)
;;     (let ((?result-object (car (proj::projection-environment-result-result proj-result))))
;;       (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;         (cpl:top-level
;;           (exe:perform (desig:an action
;;                                  (type placing)
;;                                  (arm ?arm)
;;                                  (object ?result-object))))))))

;; (defun test-place-bottle ()
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (exe:perform (desig:an action
;;                              (type placing)
;;                              (arm right))))))
