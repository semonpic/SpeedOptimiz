using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
using System.Diagnostics;
using System.Drawing;

public class Program
{
    public static void Main(string[] args)
    {
        // 测试数据：直角（90°）
        List<Segment> testSegments = new List<Segment>
        {
            new Segment { Xs = 0, Ys = 0, Xe = 100, Ye = 0, Acceleration = 10, Vmax = 200 },
            new Segment { Xs = 100, Ys = 0, Xe = 100, Ye = 100, Acceleration = 10, Vmax = 200 },
            new Segment { Xs = 100, Ys = 100, Xe = 200, Ye = 100, Acceleration = 10, Vmax = 200 }
        };

        TrajectoryPlanner planner = new TrajectoryPlanner(testSegments);
        planner.ConvertToSmoothTrajectory(tolerance: 20, minAngleDeg: 0.1);

        // 添加速度规划
        planner.PlanVelocity(maxCentripetalAcceleration: 10);

        planner.PlotTrajectory();
    }
}

public class Segment
{
    public double Xs { get; set; }
    public double Ys { get; set; }
    public double Xe { get; set; }
    public double Ye { get; set; }
    public double Acceleration { get; set; }
    public double Vmax { get; set; }
    public double StartVelocity { get; set; } = 0; // 线段起始速度
    public double EndVelocity { get; set; } = 0;   // 线段结束速度
    public double CornerMaxVelocity { get; set; } = double.MaxValue; // 拐角最大速度限制
    public bool IsArc { get; set; } = false; // 是否是圆弧段

    public double Length => Math.Sqrt(Math.Pow(Xe - Xs, 2) + Math.Pow(Ye - Ys, 2));
}

public class TrajectoryPlanner
{
    private List<Segment> originalSegments;
    private List<Segment> processedSegments;
    private List<double> cornerMaxVelocities = new List<double>(); // 存储每个拐角的最大速度

    public TrajectoryPlanner(List<Segment> originalSegments)
    {
        this.originalSegments = originalSegments;
        this.processedSegments = new List<Segment>();
    }

    // 添加角度阈值参数 (minAngleDeg)
    public void ConvertToSmoothTrajectory(double tolerance, double minAngleDeg = 5)
    {
        processedSegments.Clear();
        cornerMaxVelocities.Clear();

        if (originalSegments.Count == 0) return;
        if (originalSegments.Count == 1)
        {
            processedSegments.Add(originalSegments[0]);
            return;
        }

        // 添加第一个线段
        processedSegments.Add(originalSegments[0]);

        for (int i = 0; i < originalSegments.Count - 1; i++)
        {
            Segment current = originalSegments[i];
            Segment next = originalSegments[i + 1];

            // 计算角度（用于判断是否启用圆弧过渡）
            double angle = CalculateAngle(
                new PointD(current.Xs, current.Ys),
                new PointD(current.Xe, current.Ye),
                new PointD(next.Xe, next.Ye)
            );

            double angleDeg = angle * 180 / Math.PI;
            Console.WriteLine($"线段{i + 1}到{i + 2}的夹角: {angleDeg:F2}度");

            // 如果角度太小（接近直线），则不添加圆弧
            if (angle < minAngleDeg * Math.PI / 180 || angle > (180 - minAngleDeg) * Math.PI / 180)
            {
                Console.WriteLine($"角度{angleDeg:F2}度小于阈值{minAngleDeg}度，跳过圆弧过渡");
                // 直接连接线段
                Segment nextSegment = new Segment
                {
                    Xs = current.Xe,
                    Ys = current.Ye,
                    Xe = next.Xe,
                    Ye = next.Ye,
                    Acceleration = next.Acceleration,
                    Vmax = next.Vmax
                };
                processedSegments.Add(nextSegment);
                cornerMaxVelocities.Add(double.MaxValue); // 无拐角速度限制
                continue;
            }

            // 计算圆弧参数
            var (center, radius, arcStart, arcEnd, isValid) = MathUtils.CalculateArc(
                new PointD(current.Xs, current.Ys),
                new PointD(current.Xe, current.Ye),
                new PointD(next.Xe, next.Ye),
                maxCentripetalAccel: 10,
                epsilon: tolerance
            );

            if (isValid)
            {
                Console.WriteLine($"在点({current.Xe:F2},{current.Ye:F2})添加圆弧过渡，半径:{radius:F2}");

                // 更新当前线段的终点为圆弧起点
                processedSegments[processedSegments.Count - 1].Xe = arcStart.X;
                processedSegments[processedSegments.Count - 1].Ye = arcStart.Y;

                // 添加圆弧段
                List<Segment> arcSegments = GenerateArcSegments(
                    arcStart.X, arcStart.Y,
                    center.X, center.Y,
                    arcEnd.X, arcEnd.Y,
                    radius, tolerance,
                    current.Acceleration, current.Vmax
                );

                // 标记圆弧段
                foreach (var seg in arcSegments) seg.IsArc = true;

                processedSegments.AddRange(arcSegments);

                // 添加下一个线段（截断版）
                Segment truncatedNext = new Segment
                {
                    Xs = arcEnd.X,
                    Ys = arcEnd.Y,
                    Xe = next.Xe,
                    Ye = next.Ye,
                    Acceleration = next.Acceleration,
                    Vmax = next.Vmax
                };
                processedSegments.Add(truncatedNext);

                // 计算并保存拐角最大速度
                double cornerMaxVel = Math.Sqrt(10 * radius); // 最大向心加速度设为10 mm/s²
                cornerMaxVelocities.Add(cornerMaxVel);
                Console.WriteLine($"拐角{i + 1}最大速度: {cornerMaxVel:F2} mm/s");
            }
            else
            {
                Console.WriteLine($"无法在点({current.Xe:F2},{current.Ye:F2})添加圆弧过渡");
                // 直接连接线段
                Segment nextSegment = new Segment
                {
                    Xs = current.Xe,
                    Ys = current.Ye,
                    Xe = next.Xe,
                    Ye = next.Ye,
                    Acceleration = next.Acceleration,
                    Vmax = next.Vmax
                };
                processedSegments.Add(nextSegment);
                cornerMaxVelocities.Add(double.MaxValue); // 无拐角速度限制
            }
        }
    }

    // 前瞻后顾速度规划算法
    public void PlanVelocity(double maxCentripetalAcceleration)
    {
        if (processedSegments.Count == 0) return;

        // 初始化速度数组
        double[] v_start = new double[processedSegments.Count];
        double[] v_end = new double[processedSegments.Count];

        // 1. 正向遍历（前瞻）
        v_start[0] = 0; // 起始速度为0

        for (int i = 0; i < processedSegments.Count; i++)
        {
            Segment seg = processedSegments[i];

            // 计算当前段的最大可达结束速度
            double v_max_end = Math.Sqrt(v_start[i] * v_start[i] + 2 * seg.Acceleration * seg.Length);
            v_max_end = Math.Min(v_max_end, seg.Vmax); // 不超过段的最大速度

            // 应用拐角速度限制（如果是拐角）
            if (i < processedSegments.Count - 1 && i < cornerMaxVelocities.Count)
            {
                v_max_end = Math.Min(v_max_end, cornerMaxVelocities[i]);
            }

            v_end[i] = v_max_end;

            // 设置下一段的起始速度
            if (i < processedSegments.Count - 1)
            {
                v_start[i + 1] = v_end[i];
            }
        }

        // 2. 反向遍历（后顾）
        v_end[processedSegments.Count - 1] = 0; // 最后一段结束速度为0

        for (int i = processedSegments.Count - 1; i >= 0; i--)
        {
            Segment seg = processedSegments[i];

            // 计算当前段的最大可达起始速度
            double v_max_start = Math.Sqrt(v_end[i] * v_end[i] + 2 * seg.Acceleration * seg.Length);
            v_max_start = Math.Min(v_max_start, seg.Vmax); // 不超过段的最大速度

            // 应用拐角速度限制（如果是拐角）
            if (i < cornerMaxVelocities.Count)
            {
                v_max_start = Math.Min(v_max_start, cornerMaxVelocities[i]);
            }

            // 更新起始速度（取正向和反向计算的最小值）
            v_start[i] = Math.Min(v_start[i], v_max_start);

            // 更新前一段的结束速度
            if (i > 0)
            {
                v_end[i - 1] = Math.Min(v_end[i - 1], v_start[i]);
            }
        }

        // 3. 应用计算的速度到线段
        for (int i = 0; i < processedSegments.Count; i++)
        {
            processedSegments[i].StartVelocity = v_start[i];
            processedSegments[i].EndVelocity = v_end[i];

            // 对于圆弧段，应用拐角速度限制
            if (processedSegments[i].IsArc && i < cornerMaxVelocities.Count)
            {
                double cornerMaxVel = cornerMaxVelocities[i];
                processedSegments[i].StartVelocity = Math.Min(processedSegments[i].StartVelocity, cornerMaxVel);
                processedSegments[i].EndVelocity = Math.Min(processedSegments[i].EndVelocity, cornerMaxVel);
            }
        }
    }

    // 计算两条线段之间的夹角（弧度）
    private double CalculateAngle(PointD p0, PointD p1, PointD p2)
    {
        // 向量1: p1->p0
        double dx1 = p0.X - p1.X;
        double dy1 = p0.Y - p1.Y;

        // 向量2: p1->p2
        double dx2 = p2.X - p1.X;
        double dy2 = p2.Y - p1.Y;

        // 计算点积
        double dotProduct = dx1 * dx2 + dy1 * dy2;

        // 计算模长
        double mag1 = Math.Sqrt(dx1 * dx1 + dy1 * dy1);
        double mag2 = Math.Sqrt(dx2 * dx2 + dy2 * dy2);

        // 检查模长是否为零
        if (mag1 < 1e-10 || mag2 < 1e-10)
            return 0;

        // 计算夹角（弧度）
        double cosTheta = dotProduct / (mag1 * mag2);
        // 使用自定义Clamp方法确保在[-1,1]范围内
        cosTheta = MathUtils.Clamp(cosTheta, -1.0, 1.0);
        return Math.Acos(cosTheta);
    }

    private List<Segment> GenerateArcSegments(
        double startX, double startY,
        double centerX, double centerY,
        double endX, double endY,
        double radius, double tolerance,
        double acceleration, double vmax)
    {
        List<Segment> arcSegments = new List<Segment>();
        double startAngle = Math.Atan2(startY - centerY, startX - centerX);
        double endAngle = Math.Atan2(endY - centerY, endX - centerX);

        // 计算旋转方向
        double cross = (startX - centerX) * (endY - centerY) - (startY - centerY) * (endX - centerX);
        bool clockwise = cross < 0;

        // 调整角度范围确保正确方向
        if (clockwise)
        {
            // 顺时针
            if (endAngle > startAngle) endAngle -= 2 * Math.PI;
        }
        else
        {
            // 逆时针
            if (endAngle < startAngle) endAngle += 2 * Math.PI;
        }

        double angleRange = endAngle - startAngle;
        int numSegments = Math.Max(1, (int)Math.Ceiling(Math.Abs(angleRange) * radius / tolerance));
        double angleStep = angleRange / numSegments;

        double prevX = startX;
        double prevY = startY;

        for (int i = 1; i <= numSegments; i++)
        {
            double currentAngle = startAngle + i * angleStep;
            double currentX = centerX + radius * Math.Cos(currentAngle);
            double currentY = centerY + radius * Math.Sin(currentAngle);

            arcSegments.Add(new Segment
            {
                Xs = prevX,
                Ys = prevY,
                Xe = currentX,
                Ye = currentY,
                Acceleration = acceleration,
                Vmax = vmax
            });

            prevX = currentX;
            prevY = currentY;
        }

        return arcSegments;
    }

    public void PlotTrajectory()
    {
        // 创建图表
        Chart chart = new Chart();
        chart.Size = new Size(800, 600);
        chart.MouseWheel += Chart_MouseWheel; // 添加鼠标滚轮事件

        ChartArea chartArea = new ChartArea();
        chartArea.AxisX.ScaleView.Zoomable = true;
        chartArea.AxisY.ScaleView.Zoomable = true;
        chartArea.CursorX.IsUserEnabled = true;
        chartArea.CursorX.IsUserSelectionEnabled = true;
        chartArea.CursorY.IsUserEnabled = true;
        chartArea.CursorY.IsUserSelectionEnabled = true;

        // 设置坐标轴范围
        double minX = originalSegments.Min(s => Math.Min(s.Xs, s.Xe));
        double maxX = originalSegments.Max(s => Math.Max(s.Xs, s.Xe));
        double minY = originalSegments.Min(s => Math.Min(s.Ys, s.Ye));
        double maxY = originalSegments.Max(s => Math.Max(s.Ys, s.Ye));

        // 添加圆弧中心点
        foreach (var segment in processedSegments)
        {
            minX = Math.Min(minX, segment.Xs);
            minX = Math.Min(minX, segment.Xe);
            maxX = Math.Max(maxX, segment.Xs);
            maxX = Math.Max(maxX, segment.Xe);
            minY = Math.Min(minY, segment.Ys);
            minY = Math.Min(minY, segment.Ye);
            maxY = Math.Max(maxY, segment.Ys);
            maxY = Math.Max(maxY, segment.Ye);
        }

        double padding = Math.Max((maxX - minX) * 0.1, (maxY - minY) * 0.1);
        chartArea.AxisX.Minimum = minX - padding;
        chartArea.AxisX.Maximum = maxX + padding;
        chartArea.AxisY.Minimum = minY - padding;
        chartArea.AxisY.Maximum = maxY + padding;

        chart.ChartAreas.Add(chartArea);

        // 创建原始轨迹系列
        Series originalSeries = new Series("原始轨迹");
        originalSeries.ChartType = SeriesChartType.Line;
        originalSeries.Color = Color.Red;
        originalSeries.BorderWidth = 2;
        originalSeries.MarkerStyle = MarkerStyle.Circle;
        originalSeries.MarkerSize = 5;

        // 创建优化轨迹系列
        Series processedSeries = new Series("优化后轨迹");
        processedSeries.ChartType = SeriesChartType.Line;
        processedSeries.Color = Color.Blue;
        processedSeries.BorderWidth = 2;

        // 创建关键点系列
        Series keyPointsSeries = new Series("关键点");
        keyPointsSeries.ChartType = SeriesChartType.Point;
        keyPointsSeries.Color = Color.Green;
        keyPointsSeries.MarkerStyle = MarkerStyle.Circle;
        keyPointsSeries.MarkerSize = 8;

        // 创建圆弧中心点系列
        Series centerPointsSeries = new Series("圆弧中心");
        centerPointsSeries.ChartType = SeriesChartType.Point;
        centerPointsSeries.Color = Color.Purple;
        centerPointsSeries.MarkerStyle = MarkerStyle.Cross;
        centerPointsSeries.MarkerSize = 10;

        // 创建速度标注系列
        Series velocitySeries = new Series("速度标注");
        velocitySeries.ChartType = SeriesChartType.Point;
        velocitySeries.Color = Color.Orange;
        velocitySeries.MarkerStyle = MarkerStyle.Square;
        velocitySeries.MarkerSize = 6;
        velocitySeries.Label = "#VALY{F1}";
        velocitySeries.IsValueShownAsLabel = true;
        velocitySeries.LabelForeColor = Color.DarkBlue;
        velocitySeries.LabelAngle = -45;

        // 绘制原始轨迹
        foreach (var segment in originalSegments)
        {
            originalSeries.Points.AddXY(segment.Xs, segment.Ys);
            originalSeries.Points.AddXY(segment.Xe, segment.Ye);

            // 添加关键点
            keyPointsSeries.Points.AddXY(segment.Xs, segment.Ys);
            keyPointsSeries.Points.AddXY(segment.Xe, segment.Ye);
        }

        // 绘制优化后轨迹
        foreach (var segment in processedSegments)
        {
            processedSeries.Points.AddXY(segment.Xs, segment.Ys);
            processedSeries.Points.AddXY(segment.Xe, segment.Ye);

            // 添加速度标注（在线段中点）
            double midX = (segment.Xs + segment.Xe) / 2;
            double midY = (segment.Ys + segment.Ye) / 2;
            double avgVelocity = (segment.StartVelocity + segment.EndVelocity) / 2;
            velocitySeries.Points.AddXY(midX, midY);
            velocitySeries.Points[velocitySeries.Points.Count - 1].Tag = avgVelocity;
        }

        // 添加圆弧中心点
        for (int i = 0; i < originalSegments.Count - 1; i++)
        {
            Segment current = originalSegments[i];
            Segment next = originalSegments[i + 1];

            var (center, radius, arcStart, arcEnd, isValid) = MathUtils.CalculateArc(
                new PointD(current.Xs, current.Ys),
                new PointD(current.Xe, current.Ye),
                new PointD(next.Xe, next.Ye),
                maxCentripetalAccel: 10,
                epsilon: 5
            );

            if (isValid)
            {
                centerPointsSeries.Points.AddXY(center.X, center.Y);
            }
        }

        chart.Series.Add(originalSeries);
        chart.Series.Add(processedSeries);
        chart.Series.Add(keyPointsSeries);
        chart.Series.Add(centerPointsSeries);
        chart.Series.Add(velocitySeries);

        // 添加图例
        Legend legend = new Legend();
        legend.Docking = Docking.Bottom;
        chart.Legends.Add(legend);

        // 添加缩放控制按钮
        ToolStrip toolStrip = new ToolStrip();
        ToolStripButton zoomInButton = new ToolStripButton("放大");
        ToolStripButton zoomOutButton = new ToolStripButton("缩小");
        ToolStripButton resetButton = new ToolStripButton("重置视图");
        ToolStripButton debugButton = new ToolStripButton("调试信息");
        ToolStripButton velocityButton = new ToolStripButton("显示速度曲线");

        zoomInButton.Click += (s, e) => ZoomChart(chart, 0.8);
        zoomOutButton.Click += (s, e) => ZoomChart(chart, 1.2);
        resetButton.Click += (s, e) => ResetChartZoom(chart);
        debugButton.Click += (s, e) => ShowDebugInfo();
        velocityButton.Click += (s, e) => PlotVelocityProfile();

        toolStrip.Items.Add(zoomInButton);
        toolStrip.Items.Add(zoomOutButton);
        toolStrip.Items.Add(resetButton);
        toolStrip.Items.Add(debugButton);
        toolStrip.Items.Add(velocityButton);

        // 创建窗体
        Form form = new Form();
        form.Text = "轨迹规划结果 (使用鼠标滚轮或按钮缩放)";
        form.Size = new Size(900, 700);

        // 使用TableLayoutPanel布局
        TableLayoutPanel tableLayout = new TableLayoutPanel();
        tableLayout.Dock = DockStyle.Fill;
        tableLayout.RowCount = 2;
        tableLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));
        tableLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        // 添加控件
        tableLayout.Controls.Add(toolStrip, 0, 0);
        tableLayout.Controls.Add(chart, 0, 1);

        form.Controls.Add(tableLayout);
        Application.Run(form);
    }

    // 绘制速度曲线
    private void PlotVelocityProfile()
    {
        if (processedSegments == null || processedSegments.Count == 0)
        {
            MessageBox.Show("没有轨迹数据可显示", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            return;
        }

        Form velocityForm = new Form();
        velocityForm.Text = "速度曲线";
        velocityForm.Size = new Size(800, 600);

        Chart velocityChart = new Chart();
        velocityChart.Dock = DockStyle.Fill;

        ChartArea velocityArea = new ChartArea();
        velocityChart.ChartAreas.Add(velocityArea);

        Series velocitySeries = new Series("速度曲线");
        velocitySeries.ChartType = SeriesChartType.Line;
        velocitySeries.Color = Color.Blue;
        velocitySeries.BorderWidth = 3;

        // 计算总长度和速度
        double totalDistance = 0;
        List<double> distances = new List<double>();
        List<double> velocities = new List<double>();

        // 添加起始点
        distances.Add(0);
        velocities.Add(processedSegments[0].StartVelocity);

        foreach (var segment in processedSegments)
        {
            totalDistance += segment.Length;
            distances.Add(totalDistance);
            velocities.Add(segment.EndVelocity);

            // 添加中点速度（用于显示加速度）
            double midDistance = totalDistance - segment.Length / 2;
            double midVelocity = (segment.StartVelocity + segment.EndVelocity) / 2;
            distances.Add(midDistance);
            velocities.Add(midVelocity);
        }

        // 按距离排序
        var sortedData = distances.Zip(velocities, (d, v) => new { Distance = d, Velocity = v })
                                 .OrderBy(p => p.Distance)
                                 .ToList();

        // 添加数据点
        for (int i = 0; i < sortedData.Count; i++)
        {
            velocitySeries.Points.AddXY(sortedData[i].Distance, sortedData[i].Velocity);
        }

        velocityChart.Series.Add(velocitySeries);

        // 设置坐标轴标签
        velocityArea.AxisX.Title = "距离 (mm)";
        velocityArea.AxisY.Title = "速度 (mm/s)";
        velocityArea.AxisX.MajorGrid.LineColor = Color.LightGray;
        velocityArea.AxisY.MajorGrid.LineColor = Color.LightGray;

        velocityForm.Controls.Add(velocityChart);
        velocityForm.Show();
    }

    private void ShowDebugInfo()
    {
        string info = "=== 轨迹调试信息 ===\n";
        info += $"原始线段数量: {originalSegments.Count}\n";
        info += $"处理后线段数量: {processedSegments.Count}\n\n";

        for (int i = 0; i < originalSegments.Count - 1; i++)
        {
            Segment current = originalSegments[i];
            Segment next = originalSegments[i + 1];

            double angle = CalculateAngle(
                new PointD(current.Xs, current.Ys),
                new PointD(current.Xe, current.Ye),
                new PointD(next.Xe, next.Ye)
            );

            double angleDeg = angle * 180 / Math.PI;

            var (center, radius, arcStart, arcEnd, isValid) = MathUtils.CalculateArc(
                new PointD(current.Xs, current.Ys),
                new PointD(current.Xe, current.Ye),
                new PointD(next.Xe, next.Ye),
                maxCentripetalAccel: 10,
                epsilon: 5
            );

            info += $"拐点 {i + 1} ({current.Xe:F2}, {current.Ye:F2}):\n";
            info += $"  夹角: {angleDeg:F2}度\n";
            info += $"  圆弧有效: {isValid}\n";

            if (isValid)
            {
                info += $"  圆心: ({center.X:F2}, {center.Y:F2})\n";
                info += $"  半径: {radius:F2}\n";
                info += $"  圆弧起点: ({arcStart.X:F2}, {arcStart.Y:F2})\n";
                info += $"  圆弧终点: ({arcEnd.X:F2}, {arcEnd.Y:F2})\n";

                // 验证圆心到拐点的距离
                double distToP1 = MathUtils.Distance(center, new PointD(current.Xe, current.Ye));
                double expectedDist = radius;
                info += $"  圆心到拐点距离: {distToP1:F2} (期望: {expectedDist:F2})\n";

                // 验证圆弧起点到圆心的距离
                double distStart = MathUtils.Distance(center, arcStart);
                info += $"  圆心到起点距离: {distStart:F2} (期望: {expectedDist:F2})\n";

                // 验证圆弧终点到圆心的距离
                double distEnd = MathUtils.Distance(center, arcEnd);
                info += $"  圆心到终点距离: {distEnd:F2} (期望: {expectedDist:F2})\n";
            }

            info += "\n";
        }

        // 添加速度信息
        info += "\n=== 速度规划结果 ===\n";
        double totalDistance = 0;
        double estimatedTime = 0;

        for (int i = 0; i < processedSegments.Count; i++)
        {
            var seg = processedSegments[i];
            double segTime = CalculateSegmentTime(seg);
            estimatedTime += segTime;
            totalDistance += seg.Length;

            info += $"线段 {i + 1} ({seg.Xs:F1},{seg.Ys:F1}) -> ({seg.Xe:F1},{seg.Ye:F1}):\n";
            info += $"  长度: {seg.Length:F2} mm\n";
            info += $"  起始速度: {seg.StartVelocity:F2} mm/s\n";
            info += $"  结束速度: {seg.EndVelocity:F2} mm/s\n";
            info += $"  加速度: {seg.Acceleration} mm/s²\n";
            info += $"  预估时间: {segTime:F3} s\n";

            if (i < cornerMaxVelocities.Count)
            {
                info += $"  拐角速度限制: {cornerMaxVelocities[i]:F2} mm/s\n";
            }

            info += "\n";
        }

        info += $"总距离: {totalDistance:F2} mm\n";
        info += $"预估总时间: {estimatedTime:F3} s\n";

        MessageBox.Show(info, "调试信息", MessageBoxButtons.OK, MessageBoxIcon.Information);
    }

    // 计算线段所需时间
    private double CalculateSegmentTime(Segment seg)
    {
        if (Math.Abs(seg.StartVelocity - seg.EndVelocity) < 1e-6)
        {
            // 匀速运动
            return seg.Length / seg.StartVelocity;
        }
        else if (seg.EndVelocity > seg.StartVelocity)
        {
            // 加速运动
            double acceleration = seg.Acceleration;
            double t = (seg.EndVelocity - seg.StartVelocity) / acceleration;
            double distance = (seg.StartVelocity + seg.EndVelocity) * t / 2;

            if (distance >= seg.Length)
            {
                // 实际未达到目标速度
                double v_max = Math.Sqrt(seg.StartVelocity * seg.StartVelocity + 2 * acceleration * seg.Length);
                return (v_max - seg.StartVelocity) / acceleration;
            }
            else
            {
                // 加速到目标速度
                return t;
            }
        }
        else
        {
            // 减速运动
            double deceleration = seg.Acceleration;
            double t = (seg.StartVelocity - seg.EndVelocity) / deceleration;
            double distance = (seg.StartVelocity + seg.EndVelocity) * t / 2;

            if (distance >= seg.Length)
            {
                // 实际未减速到目标速度
                double v_min = Math.Sqrt(seg.StartVelocity * seg.StartVelocity - 2 * deceleration * seg.Length);
                return (seg.StartVelocity - v_min) / deceleration;
            }
            else
            {
                // 减速到目标速度
                return t;
            }
        }
    }

    // 鼠标滚轮缩放
    private void Chart_MouseWheel(object sender, MouseEventArgs e)
    {
        Chart chart = (Chart)sender;
        ChartArea area = chart.ChartAreas[0];

        double zoomFactor = e.Delta > 0 ? 0.8 : 1.2;
        ZoomChart(chart, zoomFactor);
    }

    // 缩放图表
    private void ZoomChart(Chart chart, double zoomFactor)
    {
        ChartArea area = chart.ChartAreas[0];
        double centerX = (area.AxisX.Minimum + area.AxisX.Maximum) / 2;
        double centerY = (area.AxisY.Minimum + area.AxisY.Maximum) / 2;

        double rangeX = (area.AxisX.Maximum - area.AxisX.Minimum) * zoomFactor;
        double rangeY = (area.AxisY.Maximum - area.AxisY.Minimum) * zoomFactor;

        area.AxisX.Minimum = centerX - rangeX / 2;
        area.AxisX.Maximum = centerX + rangeX / 2;
        area.AxisY.Minimum = centerY - rangeY / 2;
        area.AxisY.Maximum = centerY + rangeY / 2;
    }

    // 重置图表缩放
    private void ResetChartZoom(Chart chart)
    {
        double minX = originalSegments.Min(s => Math.Min(s.Xs, s.Xe));
        double maxX = originalSegments.Max(s => Math.Max(s.Xs, s.Xe));
        double minY = originalSegments.Min(s => Math.Min(s.Ys, s.Ye));
        double maxY = originalSegments.Max(s => Math.Max(s.Ys, s.Ye));

        // 添加圆弧中心点
        foreach (var segment in processedSegments)
        {
            minX = Math.Min(minX, segment.Xs);
            minX = Math.Min(minX, segment.Xe);
            maxX = Math.Max(maxX, segment.Xs);
            maxX = Math.Max(maxX, segment.Xe);
            minY = Math.Min(minY, segment.Ys);
            minY = Math.Min(minY, segment.Ye);
            maxY = Math.Max(maxY, segment.Ys);
            maxY = Math.Max(maxY, segment.Ye);
        }

        double padding = Math.Max((maxX - minX) * 0.1, (maxY - minY) * 0.1);

        ChartArea area = chart.ChartAreas[0];
        area.AxisX.Minimum = minX - padding;
        area.AxisX.Maximum = maxX + padding;
        area.AxisY.Minimum = minY - padding;
        area.AxisY.Maximum = maxY + padding;
    }
}

public static class MathUtils
{
    // 自定义Clamp方法
    public static double Clamp(double value, double min, double max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    // 自定义Clamp方法（int版本）
    public static int Clamp(int value, int min, int max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    public static double Distance(PointD p1, PointD p2)
    {
        double dx = p2.X - p1.X;
        double dy = p2.Y - p1.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    public static double DotProduct(PointD v1, PointD v2)
    {
        return v1.X * v2.X + v1.Y * v2.Y;
    }

    public static double CrossProduct(PointD v1, PointD v2)
    {
        return v1.X * v2.Y - v1.Y * v2.X;
    }

    public static (PointD center, double radius, PointD arcStart, PointD arcEnd, bool isValid) CalculateArc(
        PointD p0, PointD p1, PointD p2, double maxCentripetalAccel, double epsilon = 0.1)
    {
        // 计算向量：从拐点p1到前一点p0和到后一点p2
        PointD v1 = new PointD(p0.X - p1.X, p0.Y - p1.Y); // p1 -> p0
        PointD v2 = new PointD(p2.X - p1.X, p2.Y - p1.Y); // p1 -> p2

        // 计算向量长度
        double v1Length = Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y);
        double v2Length = Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y);

        // 检查零向量
        if (v1Length < 1e-10 || v2Length < 1e-10)
            return (new PointD(0, 0), 0, p1, p1, false);

        // 计算单位向量
        PointD u1 = new PointD(v1.X / v1Length, v1.Y / v1Length);
        PointD u2 = new PointD(v2.X / v2Length, v2.Y / v2Length);

        // 计算夹角
        double cosTheta = DotProduct(u1, u2);
        cosTheta = Clamp(cosTheta, -1.0, 1.0);
        double theta = Math.Acos(cosTheta);

        // 检查共线情况
        if (theta < 1e-6 || Math.PI - theta < 1e-6)
            return (new PointD(0, 0), 0, p1, p1, false);

        // 计算角平分线方向 (u1 + u2)
        PointD bisector = new PointD(u1.X + u2.X, u1.Y + u2.Y);
        double bisectorLength = Math.Sqrt(bisector.X * bisector.X + bisector.Y * bisector.Y);

        // 检查角平分线是否为零
        if (bisectorLength < 1e-10)
            return (new PointD(0, 0), 0, p1, p1, false);

        // 归一化角平分线
        PointD bisectorNormalized = new PointD(bisector.X / bisectorLength, bisector.Y / bisectorLength);

        // 计算圆弧半径
        double r = epsilon / (1 - Math.Sin(theta / 2));
        r = Math.Max(r, maxCentripetalAccel / (Math.PI * Math.PI));

        // 计算圆心位置：沿角平分线方向移动 d = r / sin(θ/2)
        double d = r / Math.Sin(theta / 2);
        double cx = p1.X + bisectorNormalized.X * d;
        double cy = p1.Y + bisectorNormalized.Y * d;

        // 计算圆弧起点和终点
        double offset = r / Math.Tan(theta / 2);
        PointD arcStart = new PointD(
            p1.X + u1.X * offset,
            p1.Y + u1.Y * offset
        );

        PointD arcEnd = new PointD(
            p1.X + u2.X * offset,
            p1.Y + u2.Y * offset
        );

        return (new PointD(cx, cy), r, arcStart, arcEnd, true);
    }
}

public class PointD
{
    public double X { get; set; }
    public double Y { get; set; }

    public PointD(double x, double y)
    {
        X = x;
        Y = y;
    }
}