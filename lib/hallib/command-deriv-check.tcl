proc make_sig_array {} {
  set ans [hal show signal]
  set lines [split $ans \n]
  set header_len 2
  set lines [lreplace $lines 0 [expr $header_len -1]]
  set lines [lreplace $lines end end]
  foreach line $lines {
    set howmany [scan $line "%s%s%s" fld1 fld2 fld3]
    if {$howmany == 3} {
      set signame $fld3
      lappend ::SIG(signames) $signame
      set ::SIG($signame,type)   $fld1
      set ::SIG($signame,value)  $fld2
      set ::SIG($signame,inputs) ""
      set ::SIG($signame,output) ""
      #set ::SIG($signame,ios)    ""
      continue
    }
    switch $fld1 {
      "==>" {lappend ::SIG($signame,inputs) $fld2}
      "<==" {lappend ::SIG($signame,output) $fld2}
      "<=>" {lappend ::SIG($signame,ios)    $fld2}
      default {return -code error "check-signal_usage: unrecognized <$line>"}
    }
  }
} ;# make_sig_array

proc signal_with_no_driver {pinsuffix} {
  foreach signame $::SIG(signames) {
    if {[string last "$pinsuffix" "$::SIG($signame,inputs)"] < 0} continue
    set pid_verify 0
    foreach inpin $::SIG($signame,inputs) {
      if {$::SIG($signame,output) == ""} {
         set idx [string last "$pinsuffix" "$inpin"]
         set funcname [string range "$inpin" 0 [expr $idx -1]]
         #parray ::SIG ${signame}*
         if {[string first "do-pid-calcs" [hal show funct $funcname]] >=0} {
           set pid_verify 1
         }
      }
    }
    if $pid_verify { lappend badsigs $signame }
  }
  if [info exists badsigs] {message $badsigs }
} ;# signal_with_no_driver

proc message {signames} {
  package require Tk
  foreach signame $signames {
    puts "DELETED signal with no driving pin:$signame ($::SIG($signame,inputs))"
    lappend items $signame
    hal delsig $signame
  }
  set items [string map {" " \n} $items]
  wm title . "BUG workaround"
  set txt \
"DELETED signals connected to a PID
command-deriv pin
because they have no driving pin\n
To avoid this message, delete these
signals from your HALFILE(s)"

  pack [label .message  -text "$txt"
       ] -side top -fill x -expand 1
  pack [label .items -text $items \
       ] -side top -fill x -expand 1
  pack [button .b -text Continue \
       -command {destroy .} \
       ] -side bottom
}

make_sig_array
signal_with_no_driver ".command-deriv"
