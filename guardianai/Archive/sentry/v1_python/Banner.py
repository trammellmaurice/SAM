class Banner:
    def __init__(self):
      self.image = """
&&&&@&&&&&&&&&&&&&&&&&&&&&&&&&&&&@&&&&&&@&&&&&&@&&&&&&&&&&&&&&&&&&&&&&&&&&&@&&&&
&&&&&&&@&&&&&&&&@&&&&&@&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&@&&&&@&&&&&&&&&&&&&@&&&&&&&
&&&&&&&@&&&&&&&&&&&&&&&&&&@&&&&&&@&&&&&&&@&&&&&&&&&&&&&&&&&&&@&&&&&&&&&&&&&&&&&&
&&&&&&&&&&&&&&&&&&&@&&&&&&&&&&&&&&&&&&&&&&&&&&&@&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
@&@&&&&&&&&&@&&&&&&&&&&@&&&&&&&&&&&&&&&&&&@&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&@&&&&&&
&&@&&&&&&@&&&&@&&&&&&&&&&&&&&&&&&&&&&&&&&&&&@&&&&&@&&&&&&&&&&&&&&&&&&&&@&&&&&&&&
@&&&&&&&&&&&&&&&&&&&&&&&&&&&&&@@&&&&&&&&&&&&&&&&&&&&&@@&&@&&&@&&&&&&&&&&&&&&&&&&
&&&&&@&&&&&&&&&&&&&&&&&&&&&&&&%&&&&&@&@ *@&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&@&
@@&&@&&&&&&&@&&&&&&&&&&&&&&@&@&&   @*   . *  , ,#&&&&&&@&&&&&&&&&&&@&&&&&&&@&&&&
&&&&&&&&&&&&&&&@@&&&@&&&&&&&  @&& ,../ .. .   ,. .@*&&&&&&&&&&&&&&&&&&&&&&&&@&&&
&&&&&&&@&@&&&@&&&&&&&&@&&&@ &&@  #@.@, / . &  .  *.& #&&&@&&&@&&&&&@&&&@&@&&&@&&
@&&&&@&@&&&.          &&@@&&&&, @*      .  ...@&. @&@&&&@(*..&&@&*.,,@&&&@&&&&@&
@&&&&&@&&@&%   /       &# ,&@  / @&&@ .&   (@@#,,. @&@ @@.. *@&& ,  *&&&&&&&&@@@
&&&&&&&&&&&& . @&@@ .  @, & ,,... @@. @  #.,@@ . . ,.&.@& ...,@&* ,,*&&&&&@&&&&&
&&@@@&@&&@&& . @@&@ .  &,@ . ..    /@@   %@@, .. *.,, .&@,,.,.@,.  ,,&&&@&&&&&&&
&&@&&&&&&&@%   @&@@  . &(,&&&&&&&@@&&&*..@@&@&&&&&&@&&*@@. ./*, (  **&&@&&&@&@&&
&@&&&&@&@@&&   @&&@.  .@@   %&@&&&&&@&  . &&@&&&&@& . #&@,./,**,. *,.&&&&&&&&&&&
&&@&&@&@@&&%  .@&&@ .. &@.@.@&   ,&,/@.* *&..  *.(&@*@.&@ *,  .(,,*,(&&&@&&&&&&&
@&@&&&&@@&&&   @&@@    &@. &&&* &&@&@. /.,.&&@&&,@@,@. @&( .,,.**.,**@&&&&&&&&&&
&@&&&@&@&&&%*. @%  *, ,&& .&@@.., &.&&   ,&@ @*.,. @&./&&./*.%%(& .,.&&&&&&&@&@&
@@@&&&&&&@&&  * *   .& @@@.@@@   ,& @.,,  .&(&, * @@@ &&.&&(,@ &@,//*&&&@@&@&&&@
&&&&&&&@@@&&  .,.&&&&&&&*&&@@&&%. @,&&&&&@&&@@,.&@@@@@@&&@&&@&@&&*,*/&&&&&&&&&&&
&@&&&&@&@&@&&@&@&@&&&&@&@&&&&&&&..@%@&@&@&&@@ , @@&@@&&@&@@&&@&&@@@@.&@@&&&@&&@&
&@&&@&&&@&@&@& @&&&@&&&&&&@@&@@&@/*@@&&&@&@,&.,&&&&&&@&&&&&&&&&&#@@@&&&&&@&&@&&@
&@@@@@&&&@@&@&@@&&&&@&@&@&@&&@&&@, &@@&&&&&.@. @@@&&&&&@&@&@@&@&&&&@@&@@&&&&&&&@
&@@&@&&@&@&&@&@&@@@&&&@@&&@&@@&@@@.,,@&&@@&.//&@@&&&@&&@&@@@&@@&&&&&&&&@@@&&@&&&
&&&@&@@&&@&&@&@&@&@@&&&&@@&&&@@&&@&&&@&@&@@@@@&&@@&&&&@&&&@&&@@&@&&&&@@&@@@@&&&@
@&@@@&&&&&&&@@&@@&@&&&@@&&@&&@@&&&&@@&@&&&@@@@@&@&&@&@@@&&&&&&@&&&&@@&&&@@&&@@&@
@@&@@&&@&@@@@&@&&&@&@@&@@@@@&@@&@@@@@@@&@@@&@@&@@@@@&&@&&@@@&@@&&@@&&&@&&@@@@@&@
&@&&@@&@&@@&&&@&&@@&@&&&@@@&@@@@@&@@@&&&@@&&@@@@@@@@@@@&&&@@@@&&@&&&@@&&&@@&@&@@
@&@&@@&@&&@&&@&&@@@&@@@&&@&@&&@&@&@@&@@@@@@&&&@@&&@@@&@&&@@@&&@&&&@&@@&@@&@@&&@@
&@@@@@@@@@@&@@&&&@&@@@@&@@&&&@@@@@&@@&&@@&&@@@@&@&@&@&@@@&&@@&@@@@@&@@&&@@@@@&@@
    """
    def show(self):
        print(self.image)
