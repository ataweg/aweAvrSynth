#!/usr/local/bin/perl -w
# ------------------------------------------------------------------------------
# Author              Axel Werner (AWe)
# History
#
#  2011-12-05  AWe    initial code
# ------------------------------------------------------------------------------


use strict;    # Always!
use Math::Trig;
use Switch;

use Tk;

# setup module searchpath to find the local modules,
# when script is started from anywhere

 BEGIN
 {
    my $resfile=$0;
    $resfile =~ s/(\/|\\)[^(\/|\\)]+\.pl$//;
    push @INC, $resfile;
    push @INC, $resfile."/../perl_libs";
    push @INC, $resfile."/perl_libs";
}

# ------------------------------------------------------------------------------
#  variables
# ------------------------------------------------------------------------------

my $num_samples = 256;
my $sample_frequency = 36363.6;

my $cutoff_frequency = $sample_frequency/2;

my $Wavetables ={};

my $mw;
my $canvas_height = 600;
my $canvas_width  = 800;

# %Wavetables->waveform->frequency->table
#                                 ->min
#                                 ->max
#                                 ->rms

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub createSample
{
   my $num_discretes = shift;
   my $waveform = shift;
   my $sample_index =shift;

   my $sample = 0.0;

   switch( $waveform)
   {
      case "SQU"
      {
         for( my $j = 0; $j < $num_discretes; $j++)
         {
            my $freq_index = $j*2+1;  # ungerade
            my $phase = 1.0;
            $sample += $phase * sin( $sample_index*$freq_index*2.0*pi/256)/$freq_index;
         }
      }

      case "SAW"
      {
         for( my $j = 0; $j < $num_discretes; $j++)
         {
            my $freq_index = $j+1;
            my $phase = -1.0;
            $sample += $phase * sin( $sample_index*$freq_index*2.0*pi/256)/$freq_index;
         }
      }

      case "SAW_R"
      {
         for( my $j = 0; $j < $num_discretes; $j++)
         {
            my $freq_index = $j+1;
            my $phase = 1.0;
            $sample += $phase * sin( $sample_index*$freq_index*2.0*pi/256)/$freq_index;
         }
      }

      case "TRI"
      {
         for( my $j = 0; $j < $num_discretes; $j++)
         {
            my $freq_index = $j*2+1;  # ungerade
            my $phase = ($freq_index >>1)  & 1 ? 1.0 : -1.0;
            $sample += $phase * sin( $sample_index*$freq_index*2.0*pi/256)/($freq_index*$freq_index);
         }
      }
   }

   return $sample;
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub rms
{
   my $wavetable = shift;

   my $rms = 0.0;

   for( my $i = 0; $i < $num_samples; $i++)
   {
      $rms += @$wavetable[$i]*@$wavetable[$i];
   }

   return sqrt( $rms/$num_samples);
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub min_max
{
   my $wavetable = shift;
   my $min = 0.0;
   my $max = 0.0;

   for( my $i = 0; $i < $num_samples; $i++)
   {
      $min = @$wavetable[$i] < $min ? @$wavetable[$i] : $min;
      $max = @$wavetable[$i] > $max ? @$wavetable[$i] : $max;
   }

   return ($min, $max);
}


# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub calc_num_discretes
{
   my $frequency = shift;
   my $waveform = shift;

   my $num_discretes;

#   printf( "Frequency: %9.2f\t", $frequency);

   if( $waveform eq "SQU" or $waveform eq "TRI")
   {
      $num_discretes = int(int(( $cutoff_frequency/$frequency)+1)/2);
   }
   else
   {
      $num_discretes = int( $cutoff_frequency/$frequency)+1;
   }
   return $num_discretes;
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub create_wavetable
{
   my $frequency = shift;
   my $waveform = shift;
   my $num_discretes = shift;

   my @wavetable;

#   printf( "Num freq %3d\t", $num_discretes);

   for( my $i = 0; $i < $num_samples; $i++)
   {
      $wavetable[$i] = createSample( $num_discretes, $waveform, $i);
   }

#   print "\n";

   return \@wavetable;
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

# first scale all wave form of the given class to the value range from -1.0 to 1.0.
# then scale them down to the lowest rms value

sub normalize
{
   my $waveform = shift;
   my $freq_table = $Wavetables->{ $waveform };

   foreach my $fr (keys %$freq_table)
   {
      my $wt = $freq_table->{ $fr };
      my $wavetable = $wt->{ 'table'};
      my $min = $wt->{ 'min'};
      my $max = $wt->{ 'max'};

      my $d = ($max - $min);
      my $o = ($min + $d/2);

      for( my $i = 0; $i < $num_samples; $i++)
      {
         @$wavetable[$i] = (@$wavetable[$i] + $o)/($d/2);
      }
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub normalize_rms
{
   my $waveform = shift;
   my $freq_table = $Wavetables->{ $waveform };

   my $min_rms = 1e100; # a very big number, a "googol"

   foreach my $fr (keys %$freq_table)
   {
      my $wt = $freq_table->{ $fr };
      my $wavetable = $wt->{ 'table'};
      $min_rms = $wt->{ 'rms'} < $min_rms ? $wt->{ 'rms'} : $min_rms ;
   }

   print "Normalize $waveform to min RMS $min_rms\n";

   foreach my $fr (keys %$freq_table)
   {
      my $wt = $freq_table->{ $fr };
      my $wavetable = $wt->{ 'table'};
      my $rms = $wt->{ 'rms'};


      for( my $i = 0; $i < $num_samples; $i++)
      {
         @$wavetable[$i] = @$wavetable[$i]*$min_rms/$rms;
      }
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub normalize_all
{
   foreach my $wf (keys %$Wavetables)
   {
      normalize( $wf );
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub normalize_rms_all
{

   foreach my $wf (keys %$Wavetables)
   {
      normalize_rms( $wf );
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub calc_rms_min_max_all
{
   foreach my $wf (keys %$Wavetables)
   {
      my $freq_table = $Wavetables->{ $wf };

      foreach my $fr (keys %$freq_table)
      {
         my $wavetable = $freq_table->{ $fr };

         my $rms = rms( $wavetable->{ 'table'});
         my ($min, $max) =  min_max( $wavetable->{ 'table'});

         $wavetable->{ 'min'} = $min;
         $wavetable->{ 'max'} = $max;
         $wavetable->{ 'rms'} = $rms;
      }
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub print_wavetable_real
{
   my $wavetable = shift;
   my $format = shift;

   my $max_column = 8;
   my $max_row = $num_samples/$max_column;

   for( my $row = 0; $row < $max_row ; $row++)
   {
      for( my $col = 0; $col < $max_column; $col++)
      {
         my $value = @$wavetable[$row*$max_column  + $col];

         if( $format)
         {
            printf " ",       if( $col%4==0);

            printf " % 9.6f",  $value;

            print  ", "       if( $col != $max_column-1);
         }
         else
         {
            printf " %f\n",  $value;
        }
      }

      if( $format)
      {
         print "\n" if( $row%4==3);
         print "\n" if( $row!=($max_row-1));
      }
    }
    print "\n\n";
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub print_wavetable_8bit
{
   my $wavetable = shift;
   my $scale = shift;
   my $offset = shift;
   my $format = shift;

   my $max_column = 8;
   my $max_row = $num_samples/$max_column;

   for( my $row = 0; $row < $max_row ; $row++)
   {
      for( my $col = 0; $col < $max_column; $col++)
      {
         my $value = int(@$wavetable[$row*$max_column  + $col] * $scale) + $offset;

         if( $format)
         {
            print "\t.db\t"  if( $col == 0);
            print " "        if( $col%4==0);

            printf "%4d", $value;

            print ", "       if( $col != $max_column-1);
         }
         else
         {
            printf "%d\n",  $value;
         }
      }
      if( $format)
      {
         print "\n" if( $row%4==3);
         print "\n" if( $row!=($max_row-1));
      }
    }
    print "\n\n";
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub print_wavetables
{
   foreach my $wf (sort keys %$Wavetables)
   {
      print "\n";
      print ";-------------------------------------------------------------------------------------------------------------------\n";
      print ";\n";

      my $wfs;
      my $wft;
      switch( $wf)
      {
         case "SQU"    { $wft = "SQ";    $wfs = "square"; }
         case "SAW"    { $wft = "SAW_P"; $wfs = "sawtooth"; }
         case "SAW_R"  { $wft = "SAW";   $wfs = "reverse sawtooth"; }
         case "TRI"    { $wft = "TRI";   $wfs = "triangle"; }
      }
      print ";*** Bandlimited $wfs wavetables (each table is 256 bytes long, unsigned integer)\n";

      my $freq_table = $Wavetables->{ $wf };

      my $index = 0;
      foreach my $fr (sort { $a <=> $b} keys %$freq_table)
      {
         my $wavetable = $freq_table->{ $fr };

         printf( "%s_LIMIT%d:\n", $wft, $index);
         printf "\t; base freqency: %3.2f Hz, discrets: %d, rms: %3.2f, min: %3.2f, max: %3.2f\n\n",
                $fr, $wavetable->{ 'num_discrets'}, $wavetable->{ 'rms'}, $wavetable->{ 'min'}, $wavetable->{ 'max'} ;

         my $wt = $wavetable->{ 'table'};
#         print_wavetable_real( $wt, 1);
         print_wavetable_8bit( $wt, 128, 128, 1);
         $index++;
      }
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub write_wavetables_CVS
{
   my $pathname = shift;
   my $format = shift;
   my $scale = shift;
   my $offset = shift;

   open( my $hOutput, ">:utf8", $pathname)
         or die "Can't open '$pathname for writing': $!";

   # print first headline
   foreach my $wf (sort keys %$Wavetables)
   {
      my $wfs;
      my $wft;
      switch( $wf)
      {
         case "SQU"    { $wft = "SQ";    $wfs = "square"; }
         case "SAW"    { $wft = "SAW_P"; $wfs = "sawtooth"; }
         case "SAW_R"  { $wft = "SAW";   $wfs = "reverse sawtooth"; }
         case "TRI"    { $wft = "TRI";   $wfs = "triangle"; }
      }

      my $freq_table = $Wavetables->{ $wf };
      my $first = 1;
      foreach my $fr (sort { $a <=> $b} keys %$freq_table)
      {
         if( $first)
         {
            print $hOutput  "$wfs;";
            $first = 0;
         }
         else
         {
            print $hOutput  ";";
         }
      }
      print $hOutput  ";";
   }
   print $hOutput  "\n";

   # print second headline
   foreach my $wf (sort keys %$Wavetables)
   {
      my $freq_table = $Wavetables->{ $wf };
      foreach my $fr (sort { $a <=> $b} keys %$freq_table)
      {
         printf $hOutput "f=%1.3f;", $fr;
      }
      print $hOutput  ";";
   }
   print $hOutput  "\n";

   # print the data lines of the table
   for( my $i = 0; $i < $num_samples; $i++)
   {

      foreach my $wf (sort keys %$Wavetables)
      {
         my $freq_table = $Wavetables->{ $wf };
         foreach my $fr (sort { $a <=> $b} keys %$freq_table)
         {
            my $wavetable = $freq_table->{ $fr }->{ 'table'};
            my $value = @$wavetable[$i];

            if( $format )
            {

                printf $hOutput "%d;", int($value * $scale) + $offset;
            }
            else
            {
               my $fstr = sprintf "%9.6f;", $value;
               $fstr =~ s/\./,/g;
               print $hOutput $fstr;
            }
         }
         print $hOutput  ";";
      }
      print $hOutput  "\n";
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub dump_wavetables
{
   foreach my $wf (sort keys %$Wavetables)
   {
      print "\n------------------------------------\n";
      print "Waveform $wf\n\n";

      my $freq_table = $Wavetables->{ $wf };

      foreach my $fr (sort { $a <=> $b} keys %$freq_table)
      {
         my $wavetable = $freq_table->{ $fr };

         printf "freq %9.2f rms %6.2f min %6.2f max %6.2f\n",
                $fr, $wavetable->{ 'rms'}, $wavetable->{ 'min'}, $wavetable->{ 'max'} ;
      }
   }
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

# waveform x=0..256, y= -2.0 .. +2.0 bzw. min..max
#
sub open_canvas
{
   $mw = MainWindow->new;
   my $c = $mw->Canvas(-width => $canvas_width, -height => $canvas_height);

   $c->pack;
   $c->createLine(50, $canvas_height/2, $canvas_width-50, $canvas_height/2);
   $c->createText(10, $canvas_height/2, -fill => 'blue', -text => 'X');
   $c->createLine($canvas_width/2, 50, $canvas_width/2, $canvas_height-50);
   $c->createText($canvas_width/2, 10, -fill => 'blue', -text => 'Y');

   $c->createLine(50, $canvas_height/2 +($canvas_height-100)/4, $canvas_width-50, $canvas_height/2 +($canvas_height-100)/4);
   $c->createLine(50, $canvas_height/2 -($canvas_height-100)/4, $canvas_width-50, $canvas_height/2 -($canvas_height-100)/4);

   return $c;
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

# waveform x=0..256, y= -2.0 .. +2.0 bzw. min..max
#
sub show_graph
{
   my $canvas = shift;
   my $wavetable = shift;

   my $first = 1;
   my $xp;
   my $yp;

   for (my $i = 0; $i < 256; $i++)
   {
       my $value = @$wavetable[$i];
       my $y = -$value/2*($canvas_height-100)/2+$canvas_height/2;
       my $x = ($i-128)/256*($canvas_width-100)+$canvas_width/2;

      if( $first == 0 )
      {
         $canvas->createLine( $xp, $yp, $x, $y, -fill => 'red' );
      }
      else
      {
         $first = 0;
      }
      $xp = $x;
      $yp = $y;
   }

}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub show_wave_graph
{
   my $waveform = shift;
   my $frequency = shift;

   my $wavetable = $Wavetables->{ $waveform }->{ $frequency }->{ 'table'};

   my $c = open_canvas();
   show_graph( $c, $wavetable);
   MainLoop;
}

# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

sub show_wave_graph_all
{
   my $waveform = shift;
   my $freq_table = $Wavetables->{ $waveform };

   my $c = open_canvas();
   foreach my $fr (sort keys %$freq_table)
   {
      my $wavetable = $freq_table->{ $fr }->{ 'table'};
      show_graph( $c, $wavetable);
   }

   MainLoop;
}

# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------

MAIN:
{

# start_freqency
# cutoff_frequence
# sample_frequency
# num_samples
# waveform: SQU, SAW, SAW_R, TRI

#  my @start_frequency  = (32.7, 65.4, 130.8, 261.6, 523.2, 1046.4, 2092.8, 4183.6 );  # 8 numbers
   my @start_frequency  = (  25.956,  41.203,  65.406,  103.826,  164.813,   261.624,
                            415.302, 659.251, 1046.496, 1661.209, 2637.005, 4185.984); # 12 numbers

   my @waveforms = ("SQU", "SAW", "SAW_R", "TRI");
#   my @start_frequency  = (523.2 );
#   my @waveforms = ("SQU", "SAW", "TRI");


# ----------------------------------------
# create the wave tables for all waveforms

#   create_wavetable( 4183.6, "SAW");

   foreach my $waveform (@waveforms )
   {
      foreach my $frequency (@start_frequency)
      {
         my $num_discretes = calc_num_discretes( $frequency, $waveform);
         my $wavetable_p = create_wavetable( $frequency, $waveform, $num_discretes);

         $Wavetables->{ $waveform }->{ $frequency }->{ 'table'} = $wavetable_p;
         $Wavetables->{ $waveform }->{ $frequency }->{ 'num_discrets'} = $num_discretes;
      }
   }

   calc_rms_min_max_all();
#   dump_wavetables();

   if( 1)
   {
      print "\n---------  normalize\n";

      normalize_all( $Wavetables);
      calc_rms_min_max_all();
#      dump_wavetables();

      if( 1)
      {
         print "\n---------  normalize rms \n";

         normalize_rms_all( $Wavetables);
         calc_rms_min_max_all();
         dump_wavetables();
      }
   }

    print_wavetables();
    write_wavetables_CVS("FourierWavetables.csv", 1, 128, 128 );

#   show_wave_graph( "SQU", 523.2);
#   show_wave_graph_all("SAW");

   print "\n---------  all done; paka paka\n";
}


# ------------------------------------------------------------------------------
#
# ------------------------------------------------------------------------------

# print STDERR "$_\n" for sort keys %INC;
