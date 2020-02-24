package TankSystem
  import SI = Modelica.SIunits;

  package Interfaces
    partial connector WaterFlow
      SI.Height h;
      flow SI.VolumeFlowRate Q;
      annotation(
        Icon(graphics = {Ellipse(extent = {{-100, 100}, {100, -100}}, endAngle = 360)}));
    end WaterFlow;

    connector WaterFlowP
      extends TankSystem.Interfaces.WaterFlow;
      annotation(
        Diagram,
        Icon(graphics = {Ellipse(origin = {0, 1}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 99}, {100, -101}}, endAngle = 360)}));
    end WaterFlowP;

    connector WaterFlowN
      extends TankSystem.Interfaces.WaterFlow;
      annotation(
        Icon(graphics = {Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, endAngle = 360)}));
    end WaterFlowN;
  end Interfaces;

  package Basics
    model Tank
      parameter SI.Area A = 0.005 "Cross section area";
      parameter SI.Height hStart = 1 "Starting filling level";
      parameter SI.Height hMax = 100 "Maximum filling level";
      SI.Height h "Filling level";
      SI.Volume V "Filling volume";
      SI.Volume S "Spilled volume";
      SI.VolumeFlowRate Q "Waterflow without spilling";
      SI.VolumeFlowRate Q_S "Spilling rate";
      SI.Volume VMax = A * hMax "Maximum filling volume";
      TankSystem.Interfaces.WaterFlowP waterFlowP annotation(
        Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Interfaces.WaterFlowN waterFlowN annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    initial equation
      h = hStart;
      S = 0;
    equation
      waterFlowP.h = h;
      waterFlowN.h = h;
      V = h * A;
      Q = waterFlowP.Q + waterFlowN.Q;
      der(V) = Q - Q_S;
      der(S) = Q_S;
      Q_S = if V > VMax and Q > 0 then Q else 0;
      annotation(
        Icon(graphics = {Rectangle(origin = {-1, 1}, fillColor = {170, 170, 255}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1, extent = {{-91, 91}, {91, -91}}), Rectangle(origin = {-1, -24}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Sphere, extent = {{-91, 66}, {91, -66}})}));
    end Tank;
    
      model SpillFreeTank
      extends TankSystem.Basics.Tank;
      Modelica.Blocks.Interfaces.BooleanOutput Out(start=true) annotation(
        Placement(visible = true, transformation(origin = {8, -98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 102}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
      equation
      when Q_S > 0 then
      Out = not pre(Out);
      end when;
    end SpillFreeTank;

    partial model Pipe
      parameter SI.Area A = 0.0006 "Cross section area";
      SI.Velocity v "Flow rate";
      SI.VolumeFlowRate Q "Volume flow rate";
      SI.Length d "Filling level difference between the left and the right object";
      TankSystem.Interfaces.WaterFlowP waterFlowP annotation(
        Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Interfaces.WaterFlowN waterFlowN annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      d = waterFlowP.h - waterFlowN.h;
      Q = v * A;
      waterFlowP.Q = Q;
      waterFlowN.Q = -Q;
      annotation(
        Icon(graphics = {Rectangle(fillColor = {170, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, lineThickness = 1, extent = {{-90, 20}, {90, -20}})}));
    end Pipe;

    model OpenPipe
      extends TankSystem.Basics.Pipe;
    equation
      v = Functions.flowRateToricelli(d);
      annotation(
        Icon(graphics = {Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360)}));
    end OpenPipe;
    
      model PressureValvePipe
      extends TankSystem.Basics.Pipe;
      parameter SI.Length switch = 0 "Filling level difference neccessary to open the valve";
      Boolean valve(start = false) "=true, if valve is opened.";
    equation
      when {abs(d) > switch, abs(d) < switch} then
        valve = not pre(valve);
      end when;
      v = if valve then Functions.flowRateToricelli(d) else 0;
      annotation(
        Icon(graphics = {Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360), Line(origin = {-0.21, 1.21}, points = {{-27.7933, 28.7933}, {26.2067, -29.2067}, {28.2067, -29.2067}}, thickness = 1)}));
    end PressureValvePipe;


    model TimedValvePipe
      extends TankSystem.Basics.Pipe;
      parameter SI.Time t = 100 "Switch time";
      Boolean valve(start = false) "=true, if valve is opened";
    equation
      when time > t then
        valve = not pre(valve);
      end when;
      v = if valve then Functions.flowRateToricelli(d) else 0;
      annotation(
        Icon(graphics = {Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360), Line(origin = {-3.2, 13.2}, points = {{-18.7953, 12.7953}, {3.20466, -13.2047}, {19.2047, -13.2047}}, thickness = 1)}));
    end TimedValvePipe;


    model RegulatedValvePipe
      extends TankSystem.Basics.Pipe;
      Modelica.Blocks.Interfaces.BooleanInput switch annotation(
        Placement(visible = true, transformation(origin = {0, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 60}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      Boolean valve "=true, if valve is opened.";
    equation
      valve = switch;
      v = if valve then Functions.flowRateToricelli(d) else 0;
      annotation(
        Icon(graphics = {Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360), Line(origin = {-1, 0}, points = {{-27, 30}, {27, -30}, {27, -30}}, pattern = LinePattern.Dash, thickness = 1)}));
    end RegulatedValvePipe;

    model OneWayValvePipe
      extends TankSystem.Basics.Pipe;
      Modelica.Blocks.Interfaces.BooleanInput switch annotation(
        Placement(visible = true, transformation(origin = {0, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 60}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      Boolean valve "=true, if valve is opened.";
    equation
      valve = switch;
      v = if valve then Functions.flowRateToricelli(d) else 0;
      annotation(
        Icon(graphics = {Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360), Line(origin = {-1, 0}, points = {{-27, 30}, {27, -30}, {27, -30}}, pattern = LinePattern.Dash, thickness = 1), Line(points = {{-60, 0}, {60, 0}}, color = {255, 255, 255}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15)}, coordinateSystem(initialScale = 0.1)));
    end OneWayValvePipe;


  
















  end Basics;



  package Sources
  
  partial model Feeder
     parameter SI.VolumeFlowRate Q = 0.0001 "Volume flow rate";
      TankSystem.Interfaces.WaterFlowN waterFlowN annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    annotation(
        Icon(graphics = {Rectangle(origin = {-7, 1}, fillColor = {170, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, lineThickness = 1, extent = {{-15, 19}, {97, -21}}), Rectangle(origin = {-1, 61}, fillColor = {170, 170, 255}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1, extent = {{-21, 39}, {21, -81}}), Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
    end Feeder;
  
    model ConstantFeeder extends Feeder;
      parameter SI.Time t = -1 "Stop time, negative values imply no stop time";
    protected
      Boolean inf = t < 0;
    equation
      if inf or time < t then
        waterFlowN.Q = -Q;
      else
        waterFlowN.Q = 0;
      end if;
    end ConstantFeeder;



  

    block SourceFeeder
      TankSystem.Interfaces.WaterFlowN waterFlowN annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput In annotation(
        Placement(visible = true, transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    equation
      waterFlowN.Q = -In;
      annotation(
        Diagram,
        Icon(graphics = {Rectangle(origin = {35, 0}, fillColor = {170, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, lineThickness = 1, extent = {{-55, 20}, {55, -20}}), Rectangle(origin = {0, 40}, fillColor = {170, 170, 255}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1, extent = {{-20, 60}, {20, -60}}), Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
    end SourceFeeder;

    model RegulatedFeeder extends Feeder;
      Modelica.Blocks.Interfaces.BooleanInput In annotation(
        Placement(visible = true, transformation(origin = {0, 102}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    equation
      waterFlowN.Q = if In then -Q else 0;
      
    end RegulatedFeeder;


    


  model WaterSink
      TankSystem.Interfaces.WaterFlowP waterFlowP annotation(
        Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      constant SI.Height h = 0;
    equation
      waterFlowP.h = h;
      annotation(
        Diagram,
        Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {-35, 0}, fillColor = {170, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, lineThickness = 1, extent = {{55, 20}, {-55, -20}}), Rectangle(origin = {0, -40}, fillColor = {170, 170, 255}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1, extent = {{-20, 60}, {20, -60}}), Ellipse(fillColor = {170, 170, 255}, fillPattern = FillPattern.Sphere, lineThickness = 1, extent = {{-40, 40}, {40, -40}}, endAngle = 360)}));
    end WaterSink;








  end Sources;



  package Examples
    model ThreeTankModel
      TankSystem.Sources.WaterSink waterSink annotation(
        Placement(visible = true, transformation(origin = {82, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.Tank tank1(A = 0.0027, hMax = 1.3, hStart = 1.2) annotation(
        Placement(visible = true, transformation(origin = {20, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.Tank tank2(A = 0.0027, hMax = 1.1, hStart = 0.7) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      TankSystem.Basics.Tank tank3(A = 0.0027, hMax = 0.8, hStart = 0.7) annotation(
        Placement(visible = true, transformation(origin = {-18, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.OpenPipe pipe2(A = 0.0006) annotation(
        Placement(visible = true, transformation(origin = {-62, -18}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      TankSystem.Basics.OpenPipe pipe1(A = 0.0006) annotation(
        Placement(visible = true, transformation(origin = {58, 32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      TankSystem.Sources.SourceFeeder sourceFeeder annotation(
        Placement(visible = true, transformation(origin = {-60, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Pulse pulse(amplitude = 0.0008, period = 50, width = 20) annotation(
        Placement(visible = true, transformation(origin = {-86, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.OpenPipe pipe4 annotation(
        Placement(visible = true, transformation(origin = {32, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(tank3.waterFlowN, pipe4.waterFlowP) annotation(
        Line(points = {{-8, -56}, {23, -56}}));
      connect(pipe4.waterFlowN, waterSink.waterFlowP) annotation(
        Line(points = {{41, -56}, {74, -56}}));
      connect(pulse.y, sourceFeeder.In) annotation(
        Line(points = {{-74, 82}, {-60, 82}, {-60, 64}, {-60, 64}}, color = {0, 0, 127}));
      connect(sourceFeeder.waterFlowN, tank1.waterFlowP) annotation(
        Line(points = {{-51, 52}, {12, 52}}));
      connect(pipe2.waterFlowN, tank3.waterFlowP) annotation(
        Line(points = {{-62, -27}, {-62, -56}, {-26, -56}}));
      connect(tank2.waterFlowN, pipe2.waterFlowP) annotation(
        Line(points = {{-8, 0}, {-62, 0}, {-62, -9}}));
      connect(tank2.waterFlowP, pipe1.waterFlowN) annotation(
        Line(points = {{10, 0}, {58, 0}, {58, 24}, {58, 24}}));
      connect(tank1.waterFlowN, pipe1.waterFlowP) annotation(
        Line(points = {{30, 52}, {58, 52}, {58, 42}, {58, 42}}));
      annotation(
        Icon(graphics = {Rectangle(origin = {1, -1}, extent = {{-91, 91}, {91, -91}}), Rectangle(origin = {-49, -1}, extent = {{-13, 13}, {13, -13}}), Rectangle(origin = {1, -1}, extent = {{-13, 13}, {13, -13}}), Rectangle(origin = {51, -1}, extent = {{-13, 13}, {13, -13}})}));
    end ThreeTankModel;

    model ValveModel
      TankSystem.Basics.Tank tank1(A = 0.002, hStart = 0) annotation(
        Placement(visible = true, transformation(origin = {-18, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.PressureValvePipe pressureValvePipe(A = 0.00025, switch = 1.5) annotation(
        Placement(visible = true, transformation(origin = {28, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Sources.ConstantFeeder constantFeeder1(Q = 0.0003, t = 100) annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Sources.WaterSink waterSink1 annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Sources.ConstantFeeder constantFeeder2(Q = 0.0004, t = 100) annotation(
        Placement(visible = true, transformation(origin = {-72, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.Tank tank2(A = 0.002, hStart = 0) annotation(
        Placement(visible = true, transformation(origin = {-18, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.TimedValvePipe timedValvePipe(A = 0.00025, t = 50, valve(start = true)) annotation(
        Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Sources.WaterSink waterSink2 annotation(
        Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Sources.ConstantFeeder constantFeeder3(Q = 0.00035)  annotation(
        Placement(visible = true, transformation(origin = {-72, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Basics.Tank tank3(A = 0.002, hStart = 0)  annotation(
        Placement(visible = true, transformation(origin = {-18, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Basics.RegulatedValvePipe regulatedValvePipe(A = 0.00025)  annotation(
        Placement(visible = true, transformation(origin = {30, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  TankSystem.Sources.WaterSink waterSink3 annotation(
        Placement(visible = true, transformation(origin = {70, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanPulse booleanPulse(period = 50, width = 40)  annotation(
        Placement(visible = true, transformation(origin = {2, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(booleanPulse.y, regulatedValvePipe.switch) annotation(
        Line(points = {{14, -38}, {30, -38}, {30, -66}, {30, -66}}, color = {255, 0, 255}));
      connect(regulatedValvePipe.waterFlowN, waterSink3.waterFlowP) annotation(
        Line(points = {{40, -72}, {61, -72}}));
      connect(constantFeeder3.waterFlowN, tank3.waterFlowP) annotation(
        Line(points = {{-62, -72}, {-28, -72}, {-28, -72}, {-28, -72}}));
      connect(tank3.waterFlowN, regulatedValvePipe.waterFlowP) annotation(
        Line(points = {{-8, -72}, {22, -72}, {22, -72}, {20, -72}}));
      connect(tank2.waterFlowP, constantFeeder2.waterFlowN) annotation(
        Line(points = {{-27, 0}, {-62, 0}}));
      connect(tank2.waterFlowN, timedValvePipe.waterFlowP) annotation(
        Line(points = {{-9, 0}, {19, 0}, {19, 0}, {21, 0}}));
      connect(timedValvePipe.waterFlowN, waterSink2.waterFlowP) annotation(
        Line(points = {{39, 0}, {61, 0}, {61, 0}, {59, 0}}));
  connect(constantFeeder1.waterFlowN, tank1.waterFlowP) annotation(
        Line(points = {{-61, 70}, {-26.6, 70}}));
  connect(tank1.waterFlowN, pressureValvePipe.waterFlowP) annotation(
        Line(points = {{-9, 70}, {19, 70}}));
  connect(pressureValvePipe.waterFlowN, waterSink1.waterFlowP) annotation(
        Line(points = {{37, 70}, {61, 70}, {61, 70}, {60, 70}, {60, 70}, {59, 70}}));
      annotation(
        Icon(graphics = {Rectangle(extent = {{-90, 90}, {90, -90}}), Line(origin = {-0.134232, -21.1795}, points = {{-89.8658, -68.8205}, {-53.8658, 37.1795}, {16.1342, 69.1795}, {40.1342, 37.1795}, {90.1342, 37.1795}, {90.1342, 37.1795}})}));
    end ValveModel;

    model Circular
      TankSystem.Basics.Tank tank1(hStart = 10) annotation(
        Placement(visible = true, transformation(origin = {0, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.Tank tank2(hStart = 70) annotation(
        Placement(visible = true, transformation(origin = {78, 34}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      TankSystem.Basics.Tank tank3(hStart = 30) annotation(
        Placement(visible = true, transformation(origin = {0, -66}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      TankSystem.Basics.Tank tank4(hMax = 110, hStart = 110) annotation(
        Placement(visible = true, transformation(origin = {-76, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      TankSystem.Basics.OpenPipe openPipe1 annotation(
        Placement(visible = true, transformation(origin = {46, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      TankSystem.Basics.OpenPipe openPipe3 annotation(
        Placement(visible = true, transformation(origin = {-76, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  TankSystem.Basics.TimedValvePipe timedValvePipe1 annotation(
        Placement(visible = true, transformation(origin = {-50, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  TankSystem.Basics.TimedValvePipe timedValvePipe2 annotation(
        Placement(visible = true, transformation(origin = {78, -26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    equation
      connect(timedValvePipe2.waterFlowN, tank3.waterFlowP) annotation(
        Line(points = {{78, -35}, {78, -66}, {10, -66}}));
      connect(timedValvePipe2.waterFlowP, tank2.waterFlowN) annotation(
        Line(points = {{78, -17}, {78, 26}}));
      connect(timedValvePipe1.waterFlowP, tank4.waterFlowN) annotation(
        Line(points = {{-60, 78}, {-76, 78}, {-76, 16}, {-76, 16}}));
      connect(timedValvePipe1.waterFlowN, tank1.waterFlowP) annotation(
        Line(points = {{-40, 78}, {-8, 78}, {-8, 76}, {-8, 76}}));
      connect(openPipe1.waterFlowN, tank2.waterFlowP) annotation(
        Line(points = {{55, 76}, {78, 76}, {78, 43}}));
      connect(openPipe3.waterFlowN, tank4.waterFlowP) annotation(
        Line(points = {{-76, -29}, {-76, -3}}));
      connect(tank1.waterFlowN, openPipe1.waterFlowP) annotation(
        Line(points = {{10, 76}, {37, 76}}));
      connect(tank3.waterFlowN, openPipe3.waterFlowP) annotation(
        Line(points = {{-9, -66}, {-76, -66}, {-76, -47}}));
      annotation(
        Icon(graphics = {Rectangle(origin = {0, -1}, extent = {{-90, 91}, {90, -91}}), Ellipse(origin = {-10, 10}, extent = {{-50, 50}, {70, -70}}, endAngle = 360)}),
        Diagram(coordinateSystem(initialScale = 0.1)));
    end Circular;

    model NoSpill
  Basics.OneWayValvePipe oneWayValvePipe1(A = 0.00025)  annotation(
        Placement(visible = true, transformation(origin = {46, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Sources.WaterSink waterSink1 annotation(
        Placement(visible = true, transformation(origin = {82, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  TankSystem.Basics.SpillFreeTank spillFreeTank1 annotation(
        Placement(visible = true, transformation(origin = {-4, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  TankSystem.Sources.RegulatedFeeder regulatedFeeder(Q = 0.006)  annotation(
        Placement(visible = true, transformation(origin = {-72, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
      connect(regulatedFeeder.In, spillFreeTank1.Out) annotation(
        Line(points = {{-72, 20}, {-4, 20}, {-4, 18}, {-4, 18}}, color = {255, 0, 255}));
      connect(regulatedFeeder.waterFlowN, spillFreeTank1.waterFlowP) annotation(
        Line(points = {{-63, 8}, {-13, 8}}));
    connect(oneWayValvePipe1.switch, spillFreeTank1.Out) annotation(
        Line(points = {{46, 14}, {46, 36}, {-4, 36}, {-4, 18}}, color = {255, 0, 255}));
    connect(oneWayValvePipe1.waterFlowP, spillFreeTank1.waterFlowN) annotation(
        Line(points = {{36, 8}, {5, 8}}));
      connect(oneWayValvePipe1.waterFlowN, waterSink1.waterFlowP) annotation(
        Line(points = {{56, 8}, {72, 8}, {72, 8}, {72, 8}}));
    annotation(
        Icon(graphics = {Rectangle(extent = {{-90, 90}, {90, -90}}), Rectangle(origin = {1, 0}, extent = {{-11, 10}, {11, -10}}), Line(origin = {1, 40}, points = {{-61, 0}, {59, 0}, {59, 0}}), Line(origin = {0, -40.084}, points = {{0, 20.084}, {-60, -19.916}, {60, -19.916}, {0, 20.084}, {0, 20.084}})}));end NoSpill;
  end Examples;

  package Functions
    function flowRateToricelli
      input SI.Length d;
      output SI.Velocity v;
    algorithm
      v := sign(d) * K * sqrt(abs(2 * g * d));
      annotation(
        Diagram,
        Icon(graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}), Line(origin = {-1, 0}, points = {{-59, 0}, {61, 0}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 20)}));
    end flowRateToricelli;

  end Functions;
protected
  final constant Real K = 0.2;
  final constant SI.Acceleration g = 9.81;
  annotation(
    uses(Modelica(version = "3.2.2")));
end TankSystem;
