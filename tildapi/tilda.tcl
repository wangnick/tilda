#!/bin/sh
# the next line restarts using wish \
exec wish "$0" "$@"

array set struct {acc {x y z} gyro {r p y} mag {x y z} ypr {y p r}}

set logfile "tilda.[clock format [clock seconds] -format %Y%m%d]"
set lastfile [lindex "$logfile.00.log [lsort [glob -nocomplain $logfile.*]]" end]
scan [lindex [split $lastfile "."] 2] %d lastnum
set logfile "$logfile.[format %02d [incr lastnum]].log"
puts $logfile

set err [open "|cat" r+]
set inout [open "|tilda -b >$logfile 2>@$err" w]
fconfigure $inout -buffering line
 
set valuelist [gets $err]
puts $valuelist
lappend valuelist Error ""
array set value $valuelist
foreach name {VERSION Error} {
	set noentry($name) 1
}

pack [frame .f] -fill both -expand 1
set row 0
proc row {name {noentry 1}} {
	grid [label .f.name-$name -text $name] -row $::row -column 0 -sticky nsw
	if {$noentry} {
		set w [label .f.val-$name -textvariable value($name)]
	} else {
		set w [entry .f.val-$name -textvariable ::value($name)]
		trace add variable ::value($name) write ".f.val-$name conf -fg blue; concat"
		bind $w <Return> "change $name"
	}
	grid $w -row $::row -column 1 -sticky nsw
	incr ::row
}

foreach {name val} $valuelist {
	row $name [info exists noentry($name)]
}

proc change {name} {
	.f.val-$name conf -fg black
	puts $::inout "$name $::value($name)"
}

.f.val-Error conf -fg red
tkwait visibility .
wm geometry . [winfo reqwidth .]x[winfo reqheight .]

proc refresh {} {
	update
	update idletasks
	after 1000 refresh
}
refresh
proc readable {f name} {
	set data [read $f 9999]
	if {![string length $data] || [eof $f]} {
		catch {close $f}
		exit
	}
	append ::buffer($f) $data
	while {1} {
		set i [string first "\n" $::buffer($f)]
		if {$i<0} break
		set line [string range $::buffer($f) 0 $i-1]
		puts stderr $line
		set ::value($name) $line
		set ::buffer($f) [string range $::buffer($f) $i+1 end]
	}
}

fconfigure $err -blocking false
fileevent $err readable {readable $err Error}
