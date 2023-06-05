;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :nlp)

(defvar *nlp-major-sub* nil
  "Subscriber for robot's joint state topic.")

(defvar *nlp-minor-sub* nil
  "Subscriber for robot's joint state topic.")

(defparameter *nlp-frequency* 10.0d0
  "How often to update the fluent in Hz")
(defvar *nlp-timestamp* 0.0d0
  "Timestamp of the last fluent update in secs.")

(defvar *robot-nlp-msg* (cpl:make-fluent :name :robot-nlp)
  "ROS message containing robot's current joint states.")

(defvar *command-lazy-list* '())

(defvar *command-queue* '())

(defun init-nlp-sub ()
  "Initializes *joint-state-sub*,
updating `*robot-joint-states-msg*' with frequency given in `*joint-state-frequency*'."
  (let ((update-every-?-secs (the double-float (/ 1.0d0 *nlp-frequency*))))
    (declare (double-float update-every-?-secs))
    (flet ((nlp-major-sub-cb (nlp-msg)
             (when (> (the double-float (- (roslisp:ros-time) *nlp-timestamp*))
                      update-every-?-secs)
               (roslisp:with-fields
                   (command age confidence object_type color name location)
                   nlp-msg
                 (setf *command-queue* (append *command-queue* (pairlis (list :major) (list (list (pairlis (list :location :name :color :object_type :confidence :age :command) 
                                  (list location name color object_type confidence age command))))))))
               (setf *nlp-timestamp* (the double-float (roslisp:ros-time)))))
           (nlp-minor-sub-cb (nlp-msg)
             (when (> (the double-float (- (roslisp:ros-time) *nlp-timestamp*))
                      update-every-?-secs)
               (roslisp:with-fields
                   (command age confidence object_type color name location)
                   nlp-msg
                 (setf *command-queue* (append *command-queue* (pairlis (list :minor) (list (list (pairlis (list :location :name :color :object_type :confidence :age :command)
                                  (list location name color object_type confidence age command))))))))
               (setf *nlp-timestamp* (the double-float (roslisp:ros-time))))))
      (setf *nlp-major-sub*
            (roslisp:subscribe "/robot_major_interruption"
                               "speech_processing/message_to_robot"
                               #'nlp-major-sub-cb))
      (setf *nlp-minor-sub*
            (roslisp:subscribe "/robot_minor_interruption"
                               "speech_processing/message_to_robot"
                               #'nlp-minor-sub-cb)))))

(roslisp-utilities:register-ros-init-function init-nlp-sub)

(defun next-command ()
  (pop *command-queue*))
