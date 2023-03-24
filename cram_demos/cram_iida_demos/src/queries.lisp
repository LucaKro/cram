;;;
;;; Copyright (c) 2023, Vanessa Hassouna <hassouna@cs.uni-bremen.de>
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

(in-package :demos)



(defun query-get-source-container()
  "get positions of shelfs from KnowRob and store them in a struct.
   Poses are returned in the map frame"
  (let* ((?query
	   (json-prolog:prolog-simple
	    "has_type(Tsk, 'http://www.ease-crc.org/ont/SOMA-ACT.owl#Pouring'),
             executes_task(Act, Tsk),
             has_participant(Act, Obj), 
             has_type(Role, soma:'SourceContainer'),
             triple(Obj, dul:'hasRole', Role).":package :demos))
	 (?knowrob-obj 
	   (cl::write-to-string
	    (cut:var-value '|?Obj| (cut::lazy-car ?query))))
	 (?obj-start (+ 1 (search "#" ?knowrob-obj)))
	 (?obj-trim 
		      (string-trim "'"
				   (string-trim "|"
						(subseq ?knowrob-obj ?obj-start)))))
    ?obj-trim))


(defun query-get-destination-container()
  "get positions of shelfs from KnowRob and store them in a struct.
   Poses are returned in the map frame"
  (let* ((?query
	   (json-prolog:prolog-simple
	    "has_type(Tsk, 'http://www.ease-crc.org/ont/SOMA-ACT.owl#Pouring'),
             executes_task(Act, Tsk), has_participant(Act, Obj), 
             has_type(Role, soma:'DestinationContainer'), 
             triple(Obj, dul:'hasRole', Role)." :package :demos))
	 (?knowrob-obj 
	   (cl::write-to-string
	    (cut:var-value '|?Obj| (cut::lazy-car ?query))))
	 (?obj-start (+ 1 (search "#" ?knowrob-obj)))
	  (?obj-trim 
		      (string-trim "'"
				   (string-trim "|"
						(subseq ?knowrob-obj ?obj-start)))))
    ?obj-trim))


(defun query-get-wait-duration()
  "get positions of shelfs from KnowRob and store them in a struct.
   Poses are returned in the map frame"
  (let* ((?query
	   (json-prolog:prolog-simple
	    "has_type(Tsk, 'http://www.ease-crc.org/ont/SOMA-ACT.owl#Pouring'),
             executes_task(Act, Tsk), event_interval(Act, Begin, End)." :package :demos))
	 (?knowrob-obj-begin (cut:var-value '|?Begin| (cut::lazy-car ?query)))
	 (?knowrob-obj-end (cut:var-value '|?End| (cut::lazy-car ?query))))
    (- ?knowrob-obj-end ?knowrob-obj-begin)))



(defun query-get-used-arm()
  "get positions of shelfs from KnowRob and store them in a struct.
   Poses are returned in the map frame"
  (let* ((?query
	   (json-prolog:prolog-simple
	    "has_type(Tsk, 'http://www.ease-crc.org/ont/SOMA-ACT.owl#Pouring'),
             executes_task(Act, Tsk),
             has_type(Hand, soma:'Hand'), 
             has_participant(Act,Hand).":package :demos))
	 (?knowrob-obj 
	   (cl::write-to-string
	    (cut:var-value '|?Hand| (cut::lazy-car ?query))))
	 (?obj-start (+ 1 (search "#" ?knowrob-obj)))
	  (?obj-trim 
		      (string-trim "'"
				   (string-trim "|"
						(subseq ?knowrob-obj ?obj-start)))))
    ?obj-trim))

(defun query-get-pouring-motion()
  "get positions of shelfs from KnowRob and store them in a struct.
   Poses are returned in the map frame"
  (let* ((?query
	   (json-prolog:prolog-simple
	    "has_type(Tsk, 'http://www.ease-crc.org/ont/SOMA-ACT.owl#Pouring'),
             executes_task(Act, Tsk), 
             triple(Motion,
             'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#classifies', Act)." :package :demos))
	 (?knowrob-obj 
	   (cl::write-to-string
	    (cut:var-value '|?Motion| (cut::lazy-car ?query))))
	 (?obj-start (+ 1 (search "#" ?knowrob-obj)))
	  (?obj-trim 
		      (string-trim "'"
				   (string-trim "|"
						(subseq ?knowrob-obj ?obj-start)))))
    ?obj-trim))

(defun query-get-maximum-pouring-angle()
  "get positions of shelfs from KnowRob and store them in a struct.
   Poses are returned in the map frame"
  (let* ((?query
	   (json-prolog:prolog-simple
	    "has_type(Tsk, 'http://www.ease-crc.org/ont/SOMA-ACT.owl#Pouring'),
             executes_task(Act, Tsk), has_participant(Act, Obj), 
             has_type(Role, soma:'SourceContainer'), 
             triple(Obj, dul:'hasRole', Role), 
             triple(Obj, dul:'hasRegion', Region), 
             triple(Region,
             'http://www.ease-crc.org/ont/SOMA-OBJ.owl#hasJointPositionMax', AngleMax)." :package :demos))
	 (?knowrob-obj (cut:var-value '|?AngleMax| (cut::lazy-car ?query))))
     ?knowrob-obj))


(defun query-get-minimum-pouring-angle()
  "get positions of shelfs from KnowRob and store them in a struct.
   Poses are returned in the map frame"
  (let* ((?query
	   (json-prolog:prolog-simple
	    "has_type(Tsk, 'http://www.ease-crc.org/ont/SOMA-ACT.owl#Pouring'),
             executes_task(Act, Tsk), has_participant(Act, Obj), 
             has_type(Role, soma:'SourceContainer'), 
             triple(Obj, dul:'hasRole', Role), 
             triple(Obj, dul:'hasRegion', Region), 
             triple(Region,
             'http://www.ease-crc.org/ont/SOMA-OBJ.owl#hasJointPositionMin', AngleMin).":package :demos))
	 (?knowrob-obj (cut:var-value '|?AngleMin| (cut::lazy-car ?query))))
    ?knowrob-obj))
