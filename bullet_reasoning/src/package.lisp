;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :cl-user)

(defpackage bullet-reasoning
    (:nicknames :btr)
  (:use #:common-lisp #:crs #:bt #:bt-vis #:cut)
  (:import-from #:alexandria compose curry rcurry with-gensyms copy-hash-table)
  (:import-from #:desig desig-solutions)
  (:shadow copy-object)
  (:export merge-bounding-boxes aabb with-stored-world *debug-window*
           add-debug-window add-costmap-function-object camera width
           height fov-y z-near z-far camera-axis pose gl-setup-camera
           camera-transform look-at-object-rotation
           with-rendering-to-framebuffer render-to-framebuffer
           read-pixelbuffer read-depthbuffer to-png-image add-object
           generic-cup mug plate mondamin mesh remove-object object
           name rigid-bodies rigid-body-names rigid-body world
           make-object box static-plane sphere cylinder cone
           point-cloud bt-reasoning-world invalidate-object objects
           object %object bt-reasoning-world-state robot-object links
           joint-states urdf joint-names joint-state link-names
           link-pose set-robot-state-from-tf semantic-map-object
           ensure-pose ensure-vector object-visibility
           object-visibility-percentage
           object-visibility-occluding-objects flat-color-object-proxy
           calculate-object-visibility object-visible-p
           occluding-objects simulate find-objects contact-p
           find-all-contacts find-objects-in-contact poses-equal-p
           stable-p above-p find-objects-above below-p
           find-objects-below bullet-world assert-object
           retract-object step simulate-realtime assert-object-pose
           position orientation poses-equal contact stable
           link-contacts supported-by above below visible
           occluding-objects occluding-object grasp side reachable
           blocking debug-window debug-costmap head-pointing-at
           with-current-bullet-world reach-object-ik
           set-robot-state-from-joints set-robot-state-from-tf

           robot-pan-tilt-links robot-pan-tilt-joints robot camera-frame

           event execute-event def-event
           timeline timeline-init timeline-advance
           timeline-current-world-state timeline-lookup
           execute-projection-rule timeline-apply-projection-rule
           def-projection-rule rule holds occurs at during throughout))

(desig-props:def-desig-package bullet-reasoning-designators
    (:nicknames :btr-desig)
  (:use #:common-lisp #:crs #:desig #:location-costmap
        #:btr #:designators-ros #:cut)
  (:shadowing-import-from #:desig-props at)
  (:shadowing-import-from #:btr object pose)
  (:desig-properties #:side :to #:see #:reach #:side #:name #:type
                     #:obj #:reachable-from))
