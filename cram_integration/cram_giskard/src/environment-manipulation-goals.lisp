;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :giskard)

(defun make-environment-manipulation-goal (open-or-close
                                           arm-left-or-right-or-both
                                           handle-link
                                           joint-state)
  (declare (type keyword open-or-close)
           (type (or list keyword) arm-left-or-right-or-both)
           (type symbol handle-link)
           (type (or number null) joint-state))
  (unless (listp arm-left-or-right-or-both)
    (setf arm-left-or-right-or-both (list arm-left-or-right-or-both)))
  (make-giskard-goal
   :constraints (list
                 ;; (make-prefer-base-constraint)
                 (make-open-or-close-constraint open-or-close
                                                arm-left-or-right-or-both
                                                handle-link
                                                joint-state))
   :collisions (make-constraints-vector
                (make-avoid-all-collision 0.05)
                (ecase open-or-close
                  (:open (make-allow-hand-collision
                          arm-left-or-right-or-both
                          (rob-int:get-environment-name)
                          handle-link))
                  (:close (make-allow-arm-collision
                           arm-left-or-right-or-both
                           (rob-int:get-environment-name)))))))

(defun make-grasp-bar-goal (arm-left-or-right-or-both
                            tip-grasp-axis bar-axis
                            tip-finger-axis bar-perpendicular-axis
                            bar-center
                            bar-length
                            root-link)
  (declare (type (or list keyword) arm-left-or-right-or-both)
           (type cl-transforms-stamped:vector-stamped
                 tip-grasp-axis bar-axis
                 tip-finger-axis bar-perpendicular-axis)
           (type cl-transforms-stamped:point-stamped bar-center)
           (type string root-link)
           (type number bar-length))
  (unless (listp arm-left-or-right-or-both)
    (setf arm-left-or-right-or-both (list arm-left-or-right-or-both)))
  (make-giskard-goal
   :constraints (make-constraints-vector
                 (make-prefer-base-constraint)
                 (mapcan (lambda (arm)
                           (let ((tool-frame
                                   (cut:var-value
                                    '?frame
                                    (car (prolog:prolog
                                          `(and (rob-int:robot ?robot)
                                                (rob-int:robot-tool-frame
                                                 ?robot ,arm ?frame)))))))
                             (when (cut:is-var tool-frame)
                               (error "[giskard] Tool frame was not defined."))
                             (list
                              (make-align-planes-constraint
                               cram-tf:*odom-frame*
                               tool-frame
                               bar-perpendicular-axis
                               tip-finger-axis)
                              (make-grasp-bar-constraint
                               tool-frame root-link
                               tip-grasp-axis
                               bar-axis bar-center bar-length))))
                         arm-left-or-right-or-both))))


(defun call-environment-manipulation-action (&key
                                               (open-or-close :open)
                                               (arm :right)
                                               (handle-link :iai-fridge-door-handle)
                                               joint-state
                                               action-timeout)
  (declare (type keyword open-or-close)
           (type (or keyword list) arm)
           (type symbol handle-link)
           (type (or null number) joint-state action-timeout))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal (print (make-environment-manipulation-goal
                      open-or-close arm handle-link joint-state))
       :action-timeout action-timeout)
    (ensure-goal-reached status)
    (values result status)
    ;; return the joint state, which is our observation
    ;; (joints:full-joint-states-as-hash-table)
    ))

(defun call-grasp-bar-action (&key
                                (arm
                                 :right)
                                (tip-grasp-axis
                                 (cl-transforms-stamped:make-vector-stamped
                                  (ecase arm
                                    (:left cram-tf:*robot-left-tool-frame*)
                                    (:right cram-tf:*robot-right-tool-frame*))
                                  0.0
                                  (cl-transforms:make-3d-vector 0 0 1)))
                                (tip-finger-axis
                                 (cl-transforms-stamped:make-vector-stamped
                                  (ecase arm
                                    (:left cram-tf:*robot-left-tool-frame*)
                                    (:right cram-tf:*robot-right-tool-frame*))
                                  0.0
                                  (cl-transforms:make-3d-vector 0 1 0)))
                                (bar-axis
                                 (cl-transforms-stamped:make-vector-stamped
                                  "iai_kitchen/iai_fridge_door_handle" 0.0
                                  (cl-transforms:make-3d-vector 0 0 -1)))
                                (bar-perpendicular-axis
                                 (cl-transforms-stamped:make-vector-stamped
                                  "iai_kitchen/iai_fridge_door_handle" 0.0
                                  (cl-transforms:make-3d-vector 0 1 0)))
                                (bar-center
                                 (cl-transforms-stamped:make-point-stamped
                                  "iai_kitchen/iai_fridge_door_handle" 0.0
                                  (cl-transforms:make-3d-vector 0 0 0)))
                                (bar-length
                                 0.4)
                                (root-link
                                 cram-tf:*odom-frame*)
                                action-timeout)
  (declare (type (or keyword list) arm)
           (type (or cl-transforms-stamped:vector-stamped null)
                 tip-grasp-axis bar-axis)
           (type (or cl-transforms-stamped:point-stamped null) bar-center)
           (type (or string null) root-link)
           (type (or number null) bar-length action-timeout))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal (print
                     (make-grasp-bar-goal
                      arm
                      tip-grasp-axis bar-axis
                      tip-finger-axis bar-perpendicular-axis
                      bar-center
                      bar-length root-link))
       :action-timeout action-timeout)
    (ensure-goal-reached status)
    (values result status)
    ;; return the joint state, which is our observation
    ;; (joints:full-joint-states-as-hash-table)
    ))



#+the-plan
(
 (setf (btr:joint-state (btr:get-environment-object) "iai_fridge_door_joint")
       0.0)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (giskard::call-grasp-bar-action :arm :right :bar-length 0.8)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type gripping)
              (gripper right))))
 (giskard::call-environment-manipulation-action :open-or-close :open
                                                :arm :right)
 (giskard:call-giskard-cartesian-action
  :goal-pose-right (cl-transforms-stamped:make-pose-stamped "r_gripper_tool_frame" 0.0 (cl-transforms:make-3d-vector -0.1 0 0) (cl-transforms:make-identity-rotation))
  :collision-mode :allow-all)
 (coe:on-event (make-instance 'cpoe:robot-state-changed))
 (setf (btr:joint-state (btr:get-environment-object)
                        "iai_fridge_door_joint") 1.45)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (giskard::call-environment-manipulation-action :open-or-close :close
                                                :arm :right)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (setf (btr:joint-state (btr:get-environment-object) "iai_fridge_door_joint")
       0.0)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 )

#+the-dishwasher-plan
(
 (setf (btr:joint-state (btr:get-environment-object)
                        "sink_area_dish_washer_door_joint")
       0.0)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (giskard::call-grasp-bar-action
  :arm :right
  :bar-axis (cl-transforms-stamped:make-vector-stamped
             "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
             (cl-transforms:make-3d-vector 0 1 0))
  :bar-perpendicular-axis
  (cl-transforms-stamped:make-vector-stamped
   "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
   (cl-transforms:make-3d-vector 0 0 1))
  :bar-center (cl-transforms-stamped:make-point-stamped
               "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
               (cl-transforms:make-3d-vector 0.0 0 0))
  :bar-length 0.2)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type gripping)
              (gripper right))))
 (giskard::call-environment-manipulation-action
  :open-or-close :open
  :arm :right
  :handle-link :sink-area-dish-washer-door-handle
  :joint-state (cram-math:degrees->radians 35))
 (coe:on-event (make-instance 'cpoe:robot-state-changed))
 (setf (btr:joint-state (btr:get-environment-object)
                        "sink_area_dish_washer_door_joint")
       (cram-math:degrees->radians 35))
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (giskard::call-environment-manipulation-action
  :open-or-close :close
  :arm :right
  :handle-link :sink-area-dish-washer-door-handle)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (setf (btr:joint-state (btr:get-environment-object)
                        "sink_area_dish_washer_door_joint")
       (cram-math:degrees->radians 0))
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 )


#+grasping-inside-tray-plan
(
 (giskard::call-grasp-bar-action
  :arm :right
  :bar-axis (cl-transforms-stamped:make-vector-stamped
             "iai_kitchen/sink_area_dish_washer_tray_handle_front_side" 0.0
             (cl-transforms:make-3d-vector 0 -1 0))
  :bar-perpendicular-axis
  (cl-transforms-stamped:make-vector-stamped
   "iai_kitchen/sink_area_dish_washer_tray_handle_front_side" 0.0
   (cl-transforms:make-3d-vector 0 0 -1))
  :bar-center (cl-transforms-stamped:make-point-stamped
               "iai_kitchen/sink_area_dish_washer_tray_handle_front_side" 0.0
               (cl-transforms:make-3d-vector 0.06 0 -0))
  :bar-length 0.20)
 )
