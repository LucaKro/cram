;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-itasc-pm)

(defvar *itasc-action-parser* nil)

(defvar *itasc-init-service* nil)

(defun init-itasc-integration ()
  (setf *itasc-action-parser* (actionlib:make-action-client
                               "/itasc_dummy_server"
                               "task_msgs/ItascConstrainedMotionAction"))
  (setf *itasc-init-service* (roslisp:make-service-client
                              "/InitSceneService"
                              "task_msgs/InitScene")))

(register-ros-init-function init-itasc-integration)

;;; auxiliary methods to create goal messages
(defun create-empty-itasc-action ()
  (let ((action (actionlib:make-action-goal
                    *itasc-action-parser*)))
    action))

(defun create-itasc-action (&key tasks robot-joint-weights objects)
  (actionlib:make-action-goal
      *itasc-action-parser*
    tasks tasks
    joint_weights robot-joint-weights
    objects objects))

;;; aux method to call the action
(defun perform-itasc-motion (action-goal)
  (actionlib:call-goal
   *itasc-action-parser*
   action-goal))

(defun init-itasc-parser (robot-name)
  (declare (type string robot-name))
  (when (roslisp:wait-for-service
         *itasc-init-service* 0.2)
    (roslisp:call-service
     *itasc-init-service*
     :robot (make-itasc-robot-msg
             (find-itasc-robot robot-name)))))