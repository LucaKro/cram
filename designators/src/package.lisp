;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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
;;;

(in-package :cl-user)

(desig-props:def-desig-package :cram-designators
  (:use #:common-lisp #:cram-reasoning #:cut)
  (:nicknames :desig)
  (:import-from #:alexandria
                #:curry #:rcurry #:compose #:with-gensyms
                #:format-symbol)
  (:import-from #:cram-utilities
                #:timestamp)
  (:export #:designator #:timestamp #:description #:parent #:successor
           #:designator-error
           #:*default-role*
           #:valid #:data #:equate #:desig-equal #:reference
           #:next-solution #:register-designator-class
           #:make-designator #:first-desig #:current-desig
           #:with-desig-props #:with-designators
           #:designator-solutions
           #:desig-prop-value #:*designator-pprint-description*
           #:get-equal-designators
           #:designator-id-mixin #:object-id
           #:assert-desig-binding
           #:retract-desig-binding
           #:object-designator
           #:register-object-desig-resolver
           #:resolve-object-desig
           #:action-designator #:action-desig #:action
           #:location-designator
           #:register-location-generator
           #:register-location-validation-function
           #:desig-solution #:loc-desig? #:obj-desig-location
           #:desig-reference
           #:desig-solutions
           #:loc-desig-location
           #:object #:location
           ;; #:register-designator-properties
           #:def-desig-package
           #:designator-pose
           #:designator-distance
           #:resolve-designator
           ;; Properties & prolog related stuff
           #:desig-solution
           #:desig-prop #:desig-class
           #:desig-timestamp #:desig-description
           #:desig-valid #:desig-value
           #:desig-location-prop
           #:desig
           #:trajectory-desig?)
  (:desig-properties #:obj #:location #:object #:pose #:of #:at))
