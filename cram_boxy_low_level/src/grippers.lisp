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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :boxy-ll)

(defparameter *gripper-minimal-position* 0.0 "in millimeters")
(defparameter *gripper-maximal-position* 100.0 "in millimeters")

(defvar *gripper-publishers* '(:left nil :right nil)
  "ROS publisher for Boxy gripper driver on goal_position message.")

(defun init-gripper-position-publishers ()
  (setf (getf *gripper-publishers* :left)
        (roslisp:advertise "left_arm_gripper/goal_position" "iai_wsg_50_msgs/PositionCmd"))
  (setf (getf *gripper-publishers* :right)
        (roslisp:advertise "right_arm_gripper/goal_position" "iai_wsg_50_msgs/PositionCmd")))

(defun destroy-gripper-position-publishers ()
  (setf *gripper-publishers* '(:left nil :right nil)))

(roslisp-utilities:register-ros-init-function init-gripper-position-publishers)
(roslisp-utilities:register-ros-cleanup-function destroy-gripper-position-publishers)

(defun ensure-gripper-input-parameters (action-type position effort)
  (let ((position
          (cond
            (position
             (cond
               ((< position *gripper-minimal-position*)
                (roslisp:ros-warn (gripper-action)
                                  "POSITION (~a) cannot be < 0.0. Clipping."
                                  position)
                *gripper-minimal-position*)
               ((> position *gripper-maximal-position*)
                (roslisp:ros-warn (gripper-action)
                                  "POSITION (~a) shouldn't be > 0.085. Clipping."
                                  position)
                *gripper-maximal-position*)
               (t
                position)))
            (action-type
             (ecase action-type
               (:open *gripper-maximal-position*)
               (:close *gripper-minimal-position*)
               (:grip *gripper-minimal-position*)))))
        (effort
          (or effort
              (cond
                (position 30.0)
                (action-type (ecase action-type
                               (:open 30.0)
                               (:close 30.0) ; because of stupid gazebo
                               (:grip 15.0)))))))
    (values position effort)))

(defun move-gripper-joint (&key action-type left-or-right goal-position effort)
  (declare (type (or keyword list) left-or-right)
           (type (or null number) goal-position effort))
  (multiple-value-bind (goal-position effort)
      (ensure-gripper-input-parameters action-type goal-position effort)
    (format t "action-type: ~a~%left-or-right: ~a~%goal-position: ~a~%effort: ~a~%"
            action-type left-or-right goal-position effort)
    (roslisp:publish
     (getf *gripper-publishers* left-or-right)
     (roslisp::make-message
      'iai_wsg_50_msgs-msg:PositionCmd
      :pos goal-position
      :speed 50.0
      :force effort))))

;; speed can be up to 60
;; force can be up to 50
;; /left_arm_gripper /right_arm_gripper
