
use strict;

sub translatename
{
	my $original = $_[0];
	my $newname;

	if ($original =~ m/Left(.*)/)
	{
		$newname = "Right$1";
	}
	elsif ($original =~ m/L(.*)/)
	{
		$newname = "R$1";
	}
	else
	{
		$newname = "Mirrored$original"
	}
	return $newname;
}

my $autoname = @ARGV[0];
my $dirname = "./src/main/deploy/pathplanner";
open (my $auto, "<", "$dirname/autos/$autoname.auto") or die $!;
my $text = do {local $/; <$auto> };
close $auto;


#print $text;

my @relevantpaths;

while ($text =~ /"pathName": "(.*)"/g)
{
	push @relevantpaths, $1;
}

print join ", ", @relevantpaths;

print "\n";

my $newname = translatename $autoname;

$text =~ s/"pathName": "(.*)"/"\"pathName\": \"" . translatename $1 . "\""/ge;

print $newname;
open (my $newauto, ">", "$dirname/autos/$newname.auto") or die $!;
print $newauto $text;


foreach my $path (@relevantpaths)
{
	open my $in, "<", "$dirname/paths/$path.path";
	my $newpathname = translatename $path;

	open my $out, ">", "$dirname/paths/$newpathname.path";

	my $contents = do {local $/; <$in>};

	close $in;

	$contents =~ s/"y": (.*)/"\"y\": " . (8.069326 - $1)/ge;
	$contents =~ s/"rotation": (.*)/"\"rotation\": " . (-$1)/ge;

	$contents =~ s/"folder": "(.*)"/"\"folder\": \"" . translatename $1 . "\""/ge;
	print $out $contents;

	close $out;
}
